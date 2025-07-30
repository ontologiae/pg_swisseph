/*
 *  swisseph_planets.c – extension PostgreSQL 17+
 *
 *  Compilation : via PGXS – voir Makefile.
 *  Dépend :  libswisseph-dev (2.10)  et  PostgreSQL server headers.
 *
 *  Résultat :
 *     sw_planet_positions(ts timestamptz, latitude float8, longitude float8)
 *        RETURNS TABLE
 *        (
 *          ts        timestamptz,   -- horodatage d’appel (inchangé)
 *          idplanet  int4,          -- identifiant SwissEph ou point Angular
 *          lon       float8,        -- longitude écliptique (° 0-360)
 *          lat       float8,        -- latitude  (°)
 *          dist      float8         -- distance (AU)
 *        )
 *
 *  ASC (=10001), DSC (=10002), MC (=10003), IC (=10004)
 *
 *  Auteur :  Pierre-Alexandre Voye et le Chat
 *  Licence : même que PostgreSQL
 */

/* ---- Évite la redéfinition d’int64 par Swiss-Ephemeris -------------- */
#define int64 swe_int64_t          /* on rebaptise int64 le temps d’un #include */
#include <swephexp.h>              /* ancien #include déplacé ici               */
#undef  int64                      /* on rend le vrai nom int64 à PostgreSQL    */
#include <math.h>
#include <string.h>
#include "postgres.h"
#include "fmgr.h"
#include "funcapi.h"
#include "utils/timestamp.h"
#include "utils/geo_decls.h"

PG_MODULE_MAGIC;

/* ------------------------------------------------------------------ */
/*  Constantes internes                                               */
/* ------------------------------------------------------------------ */
#define N_PLANETS    21        /* Sun … Pluto (SwissEph 0–9)     */
#define N_URANIAN  9         /* Ceres, Pallas, Juno, Vesta, Chiron, Pholus, Varuna, Orcus, Haumea */
#define N_ANGULAR     4         /* ASC, DSC, MC, IC               */
#define NB_RESULTS   (N_PLANETS + N_ANGULAR + N_URANIAN)

/* Identifiants arbitraires (> 10000) pour les angles */
#define ID_ASC 10001
#define ID_DSC 10002
#define ID_MC  10003
#define ID_IC  10004

/* Structure conservée entre deux appels d’un SRF */
typedef struct
{
    TimestampTz ts;
    int32   nrows;             /* = NB_RESULTS                    */
    int32   current;           /* index courant 0…nrows-1         */

    double  results[NB_RESULTS][4];
    /* [i][0] = idplanet
       [i][1] = lon (deg)
       [i][2] = lat (deg)
       [i][3] = dist (AU) – inutilisé pour ASC/… -> 0  */
} calc_state;

/* ------------------------------------------------------------------ */
/*  Conversion Timestamp → Julian Day UT                              */
/* ------------------------------------------------------------------ */
static double
timestamp_to_jdut(TimestampTz ts)
{
    struct pg_tm tm;
    fsec_t  fsec;
    int     tz;
    double  jut, jd_ut;
    int     rc;

    rc = timestamp2tm(ts, &tz, &tm, &fsec, NULL, NULL);
    if (rc != 0)
        ereport(ERROR, (errmsg("conversion timestamp failed")));

    /* Postgres renvoie la date en UTC quand tz est fourni                            */
    /* swe_utc_to_jd :  y,m,d,h,m,s  →  jd_et, jd_ut                                   */
    double dret[2];
    char serr[AS_MAXCH];          /* buffer erreur SwissEph */
    int ret = swe_utc_to_jd(tm.tm_year, tm.tm_mon, tm.tm_mday,
		    tm.tm_hour, tm.tm_min,
                        (double)tm.tm_sec + (double)fsec/1000000.0,
                        SE_GREG_CAL, dret, serr);
    ereport(INFO,errmsg("LOG swe_utc_to_jd : %f", dret[1]));
    if (ret == ERR)
	    ereport(ERROR,
            (errmsg("swe_utc_to_jd failed: %s", serr)));

    jd_ut = dret[1];          /* 0 = ET, 1 = UT */
    return jd_ut;
}

/* ------------------------------------------------------------------ */
/*  Pré-calcul de toutes les positions pour un appel                  */
/* ------------------------------------------------------------------ */
static void
compute_positions(calc_state *st, double lat_deg, double lon_deg)
{
    int i;
    double jd_ut = timestamp_to_jdut(st->ts);
    int32  iflag = SEFLG_SWIEPH | SEFLG_SPEED | SEFLG_TOPOCTR | SE_SPLIT_DEG_ZODIACAL;    /* éphémerides suisses, vitesses */

    char serr[AS_MAXCH];          /* buffer erreur SwissEph */

    swe_set_topo(lon_deg, lat_deg, 0.0); /* position géographique pour les calculs, au niveau de la mer */
    /* 1) Planètes 0–9 ------------------------------------------------ */
    for (i = 0; i < N_PLANETS; i++)
    {
        double xret[6];
        if (swe_calc_ut(jd_ut, i, iflag, xret, serr) == ERR)
            ereport(ERROR, (errmsg("we_calc_ut error: %s", serr)));

        st->results[i][0] = (double)i;      /* idplanet 0 … 9           */
        st->results[i][1] = swe_degnorm(xret[0]);  /* longitude          */
        st->results[i][2] = xret[1];              /* latitude            */
        st->results[i][3] = xret[3];              /* distance speed (AU)       */
    }

   for (i = 40; i < 49; i++) // Astéroides 40–48
    {
        double xret[6];
        if (swe_calc_ut(jd_ut, i, iflag, xret, serr) == ERR)
            ereport(ERROR, (errmsg("we_calc_ut error: %s", serr)));
	// on met un -19 pour commencer à 21 jusqu'à 29
        st->results[i-19][0] = (double)i;      /* idplanet 0 … 9           */
        st->results[i-19][1] = swe_degnorm(xret[0]);  /* longitude          */
        st->results[i-19][2] = xret[1];              /* latitude            */
        st->results[i-19][3] = xret[3];              /* distance speed (AU)       */
    }


    /* 2) Ascendant / MC ---------------------------------------------- */
    double cusps[13];      /* non utilisé ici, mais requis       */
    double ascmc[10];
    if (swe_houses_ex(jd_ut, iflag, lat_deg, lon_deg, 'P', cusps, ascmc) == ERR)
        ereport(ERROR, (errmsg("swe_houses_ex failed")));

    /* Ascendant, MC, leurs oppositions */
    double asc = swe_degnorm(ascmc[SE_ASC]);
    double mc  = swe_degnorm(ascmc[SE_MC]);
    double dsc = swe_degnorm(asc + 180.0);
    double ic  = swe_degnorm(mc  + 180.0);

    int base = N_PLANETS;// + 30 car on a les astéroides 40–48
    /* ASC */
    st->results[base+0][0] = ID_ASC;
    st->results[base+0][1] = asc;
    st->results[base+0][2] = 0.0;
    st->results[base+0][3] = 0.0;

    /* DSC */
    st->results[base+1][0] = ID_DSC;
    st->results[base+1][1] = dsc;
    st->results[base+1][2] = 0.0;
    st->results[base+1][3] = 0.0;

    /* MC */
    st->results[base+2][0] = ID_MC;
    st->results[base+2][1] = mc;
    st->results[base+2][2] = 0.0;
    st->results[base+2][3] = 0.0;

    /* IC */
    st->results[base+3][0] = ID_IC;
    st->results[base+3][1] = ic;
    st->results[base+3][2] = 0.0;
    st->results[base+3][3] = 0.0;
}

/* ------------------------------------------------------------------ */
/*  Fonction SRF visible depuis SQL                                   */
/* ------------------------------------------------------------------ */
PG_FUNCTION_INFO_V1(sw_planet_positions);

Datum
sw_planet_positions(PG_FUNCTION_ARGS)
{
    FuncCallContext   *funcctx;
    calc_state        *st;

    /* première entrée ------------------------------------------------ */
    if (SRF_IS_FIRSTCALL())
    {
        MemoryContext   oldcontext;

        /* vérification des arguments */
        if (PG_ARGISNULL(0) || PG_ARGISNULL(1) || PG_ARGISNULL(2))
            ereport(ERROR, (errmsg("NULL not allowed")));

        funcctx = SRF_FIRSTCALL_INIT();
        oldcontext = MemoryContextSwitchTo(funcctx->multi_call_memory_ctx);

        /* -------------------- allocate / store state ---------------- */
        st = (calc_state *) palloc0(sizeof(calc_state));
        st->ts = PG_GETARG_TIMESTAMPTZ(0);
        st->nrows = NB_RESULTS;
        st->current = 0;

        double lat_deg = PG_GETARG_FLOAT8(1);
        double lon_deg = PG_GETARG_FLOAT8(2);

        swe_set_ephe_path("/usr/share/libswe/ephe");   /* path paquet Debian */

        compute_positions(st, lat_deg, lon_deg);

        funcctx->user_fctx = (void *) st;

        /* description du tuple retourné (5 colonnes) */
        TupleDesc tupledesc;
        tupledesc = CreateTemplateTupleDesc(5);
        TupleDescInitEntry(tupledesc, (AttrNumber)1, "ts",
                           TIMESTAMPTZOID, -1, 0);
        TupleDescInitEntry(tupledesc, (AttrNumber)2, "idplanet",
                           INT4OID, -1, 0);
        TupleDescInitEntry(tupledesc, (AttrNumber)3, "lon",
                           FLOAT8OID, -1, 0);
        TupleDescInitEntry(tupledesc, (AttrNumber)4, "lat",
                           FLOAT8OID, -1, 0);
        TupleDescInitEntry(tupledesc, (AttrNumber)5, "distspeed",
                           FLOAT8OID, -1, 0);

        funcctx->tuple_desc = BlessTupleDesc(tupledesc);

        MemoryContextSwitchTo(oldcontext);
    }

    /* appels suivants ------------------------------------------------- */
    funcctx = SRF_PERCALL_SETUP();
    st      = (calc_state *) funcctx->user_fctx;

    if (st->current < st->nrows)
    {
        Datum       values[5];
        bool        nulls[5] = {false,false,false,false,false};

        values[0] = TimestampGetDatum(st->ts);
        values[1] = Int32GetDatum((int32) st->results[st->current][0]);
        values[2] = Float8GetDatum(st->results[st->current][1]);
        values[3] = Float8GetDatum(st->results[st->current][2]);
        values[4] = Float8GetDatum(st->results[st->current][3]);

        HeapTuple   tuple = heap_form_tuple(funcctx->tuple_desc,
                                            values, nulls);
        st->current++;

        SRF_RETURN_NEXT(funcctx, HeapTupleGetDatum(tuple));
    }
    else
    {
        SRF_RETURN_DONE(funcctx);
    }
}

