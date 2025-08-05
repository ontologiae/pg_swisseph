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


#define N_ANGULAR     4         /* ASC, DSC, MC, IC               */
#define N_ASTEROIDS 3         /* Proserpine, Apollon, Bacchus      */
#define BODIES_COUNT 32
#define PARTS_COUNT  1 // "Part du monde"
#define NB_RESULTS   (BODIES_COUNT + N_ANGULAR  + N_ASTEROIDS+ PARTS_COUNT) /* nb de résultats retournés par appel, et donc taille du tablea de la structure renvoyée*/

/* Identifiants arbitraires (> 10000) pour les angles */
#define ID_ASC 10001
#define ID_DSC 10002
#define ID_MC  10003
#define ID_IC  10004
#define ID_PART 10005 // Part du monde


/*

 
 SE_SUN          0        SE_MOON         1       
 SE_MERCURY      2        SE_VENUS        3       
 SE_MARS         4        SE_JUPITER      5       
 SE_SATURN       6        SE_URANUS       7       
 SE_NEPTUNE      8        SE_PLUTO        9       
 SE_MEAN_NODE    10       SE_TRUE_NODE    11
 SE_MEAN_APOG    12       SE_OSCU_APOG    13    
 SE_EARTH        14       SE_CHIRON       15      
 SE_PHOLUS       16       SE_CERES        17      
 SE_PALLAS       18       SE_JUNO         19      
 SE_VESTA        20       SE_INTP_APOG    21      
 SE_INTP_PERG    22     SE_NPLANETS     23      

 SE_PLMOON_OFFSET   9000
 SE_AST_OFFSET   10000
 SE_VARUNA   (SE_AST_OFFSET + 20000)

 SE_FICT_OFFSET  	40
 SE_FICT_OFFSET_1  	39
 SE_FICT_MAX  	       999 
 SE_NFICT_ELEM           15

 SE_COMET_OFFSET 1000

 SE_NALL_NAT_POINTS      (SE_NPLANETS + SE_NFICT_ELEM)

 SE_CUPIDO       	40  SE_HADES        	41
 SE_ZEUS         	42  SE_KRONOS       	43
 SE_APOLLON      	44  SE_ADMETOS      	45
 SE_VULKANUS     	46  SE_POSEIDON     	47
 SE_ISIS         	48  SE_NIBIRU       	49
 SE_NEPTUNE_LEVERRIER    5
 SE_NEPTUNE_ADAMS        52  SE_PLUTO_LOWELL         53
 SE_PLUTO_PICKERING      54 SE_VULCAN      		55
 SE_WHITE_MOON  	 56  SE_PROSERPINA  		57
 SE_WALDEMATH  		58
 * */


#define YEAR_EARTH 365.2421905166
#define PERIOD_BACCHUS      720.01539479335631160550                   // années        
#define ROT_SPEED_BACCHUS   (360.0 / (PERIOD_BACCHUS * YEAR_EARTH))     /* ° par jour */

#define JD0_BACCHUS 2203540.5                 /* 0° Bélier pour Bacchus */
#define JD0_APOLLON  2355952.330904602        /* 0° Bélier pour Apollon */

#define RADIUS_BACCHUS      pow(PERIOD_BACCHUS, 2.0/3.0)

#define PERIOD_APOLLON      612.84
#define RADIUS_APOLLON      pow(PERIOD_APOLLON, 2.0/3.0)
#define ROT_SPEED_APOLLON   360.0/(612.84*YEAR_EARTH)     /* ° par jour */

#define PERIOD_PROSERPINE   374.99706648423204722107
#define RADIUS_PROSERPINE   pow(PERIOD_PROSERPINE, 2.0/3.0)
#define ROT_SPEED_PROSERPINE   0.00264051506438185377 //0.002628413515156272     /* ° par jour */
#define JD0_PROSERPINE  2284136.888390 //= 1914/12/07, 10:12:31.967985928058624h// 2420246.549375 //2282637.2446808508     /* 0° Bélier pour Proserpine */

// Calculs pour Apollon, Proserpine/Koré, Bacchus

static double norm360(double a) {
	a = fmod(a, 360.0);
	return (a < 0.0) ? a + 360.0 : a;
}

static double bacchus_helio(double jd) {
	double lon;
	if (jd > JD0_BACCHUS)
		lon = 360.0 - (jd - JD0_BACCHUS) * ROT_SPEED_BACCHUS;
	else
		lon = 360.0 - (JD0_BACCHUS - jd) * ROT_SPEED_BACCHUS;

	return norm360(lon);
}

static double apollon_helio(double jd) {
	double lon;
	if (jd > JD0_APOLLON)
		lon = norm360( (jd - JD0_APOLLON) * ROT_SPEED_APOLLON);
	else
		lon = norm360( (JD0_APOLLON - jd) * ROT_SPEED_APOLLON);

	return norm360(lon);
}

static double proserpine_helio(double jd) {
	double lon;
	if (jd > JD0_PROSERPINE)
		lon = norm360( (jd - JD0_PROSERPINE) * ROT_SPEED_PROSERPINE);
	else
		lon = norm360( (JD0_PROSERPINE - jd) * ROT_SPEED_PROSERPINE);
	//ereport(INFO,errmsg("LOG proserpine_helio jd=%f ; jd0=%f ; lon=%f, norm lon=%f", jd, JD0_PROSERPINE, lon, norm360(lon) ));
	return norm360(lon);
}

static double lambda_geo_fast(double jd, double body_lat_helio,  double body_radius ) {
	char   serr[256];
	double xx_earth[6];

	/* Terre héliocentrique, plan de l’écliptique */
	swe_calc(jd, SE_EARTH, SEFLG_SWIEPH | SEFLG_SPEED | SEFLG_HELCTR  | SEFLG_SWIEPH, xx_earth, serr);

	double lambda_earth = norm360(xx_earth[0]);      /* degrés       */

	/* ---------------- formule vectorielle dans le plan ------------ */
	const double D2R = M_PI / 180.0;
	double lam_h = body_lat_helio;

	double x = body_radius * cos(lam_h * D2R) - cos(lambda_earth * D2R);
	double y = body_radius * sin(lam_h * D2R) - sin(lambda_earth * D2R);

	double lam_g = atan2(y, x) / D2R;
	if (lam_g < 0) lam_g += 360.0;
	//ereport(INFO,errmsg("LOG lambda_geo_fast lambda_earth=%f ;  jd=%f ; lambda_helio=%f ; geocentric=%f;",lambda_earth, jd, body_lat_helio, lam_g ));


	return lam_g;          /* longitude géocentrique (°) */

}

/*static double lambda_geo_fast(double jd, double lambda_helio, int32 iflag_se) {
	double xx_earth[6];              // sortie SE       
	char   serr[256];

	// λ⊕ héliocentrique (ellipse de la Terre) 
	swe_calc(jd, SE_EARTH, iflag_se | SEFLG_HELCTR , xx_earth, serr);
	double lambda_earth = norm360(xx_earth[0]);    // longitude Terre 

	ereport(INFO,errmsg("LOG lambda_geo_fast lambda_earth=%f ;  jd=%f ; lambda_helio=%f ; geocentric=%f",lambda_earth, jd, lambda_helio, norm360(lambda_helio +  lambda_earth) ));

	return norm360(lambda_helio + 180.0 - lambda_earth);


}*/


/*void bacchus_geo_vector(double jd, int32 iflag_se, double *plon, double *plat, double *pdist) {
	double xx_ast[6], xx_earth[6], rg[3];
	char   serr[256];

	// 2063 = numéro MPC de Bacchus ; dans SE : SE_AST_OFFSET + nbr   
	int32 ipl_bacchus = SE_AST_OFFSET + 2063;

	// Héliocentrique écliptique rect. (x,y,z) du corps et de la Terre 
	swe_calc(jd, ipl_bacchus, iflag_se | SEFLG_HELCTR | SEFLG_ECL, xx_ast, serr);
	swe_calc(jd, SE_EARTH,     iflag_se | SEFLG_HELCTR | SEFLG_ECL, xx_earth, serr);

	// Passage hélioc. → géoc. : soustraction vecteurs                 
	rg[0] = xx_ast[0] - xx_earth[0];
	rg[1] = xx_ast[1] - xx_earth[1];
	rg[2] = xx_ast[2] - xx_earth[2];

	double lon = atan2(rg[1], rg[0]) * 180.0 / M_PI;
	if (lon < 0) lon += 360.0;

	double lat  = atan2(rg[2], hypot(rg[0], rg[1])) * 180.0 / M_PI;
	double dist = sqrt(rg[0]*rg[0] + rg[1]*rg[1] + rg[2]*rg[2]);

	if (plon)  *plon  = lon;
	if (plat)  *plat  = lat;
	if (pdist) *pdist = dist;
}*/


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
static double timestamp_to_jdut(TimestampTz ts) {
    struct pg_tm tm;
    fsec_t  fsec;
    int     tz;
    double  jd_ut;
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
    //ereport(INFO,errmsg("LOG swe_utc_to_jd : %f", dret[1]));
    if (ret == ERR)
	    ereport(ERROR,
            (errmsg("swe_utc_to_jd failed: %s", serr)));

    jd_ut = dret[1];          /* 0 = ET, 1 = UT */
    return jd_ut;
}

/* ------------------------------------------------------------------ */
/*  Pré-calcul de toutes les positions pour un appel                  */
/* ------------------------------------------------------------------ */
static void compute_positions(calc_state *st, double lat_deg, double lon_deg) {
    int i;
    double jd_ut = timestamp_to_jdut(st->ts);
    int32  iflag = SEFLG_JPLEPH | SEFLG_SPEED | SEFLG_TOPOCTR ;    /* éphémerides suisses, vitesses */

    char serr[AS_MAXCH];          /* buffer erreur SwissEph */
    float moon, sun;

    int bodies[BODIES_COUNT] = {
	    SE_SUN,SE_MOON,SE_MARS,SE_MERCURY,SE_JUPITER,SE_VENUS,SE_SATURN,SE_URANUS,SE_NEPTUNE,SE_PLUTO // 10 with moon
		,SE_VESTA,SE_VARUNA
		,SE_VULCAN,SE_CHIRON
		,SE_CERES,SE_JUNO,SE_MEAN_APOG,SE_PHOLUS,SE_PALLAS,SE_POSEIDON // 6
		,SE_MEAN_NODE, SE_TRUE_NODE // 2
		,SE_AST_OFFSET + 136199 // Eris
		, SE_AST_OFFSET + 90377 // Sedna
		, SE_AST_OFFSET + 20000 // Varuna
		//, SE_AST_OFFSET + 2063 // Bacchus
		//, SE_AST_OFFSET + 2060 // Apollon
		//, SE_AST_OFFSET + 311999 // Koré
		, SE_AST_OFFSET + 136108 // Haumea
		, SE_AST_OFFSET + 90482 // Orcus
		, SE_AST_OFFSET + 50000 // Quaoar
		, SE_AST_OFFSET + 28978 // Ixion
		, SE_AST_OFFSET + 136472 // Makemake
		, SE_AST_OFFSET +  16 // Psyché
		, SE_AST_OFFSET +  7066 //Nessus
				       //
					// TODO : AMOR, SEDNA, Haumea, Vesta, Isis, Nessus,  Quaoar, Ixion,  Psyché, Poseidon
					// MakeMake, Orcus, Haumea,  
    };
   //Changer le makefile pour qu'il charge les fichiers se1 dans le répertoire ephe qu'on définira et modif le source C pour mettre le bon PATH 


    swe_set_topo(lon_deg, lat_deg, 0.0); /* position géographique pour les calculs, au niveau de la mer */
    /* 1) Planètes 0–9 ------------------------------------------------ */
    for (i = 0; i < BODIES_COUNT; i++) {

        double xret[6];
        if (swe_calc_ut(jd_ut, bodies[i], iflag, xret, serr) == ERR)
            ereport(ERROR, (errmsg("we_calc_ut error: %s", serr)));
	//ereport(INFO,errmsg("LOG swe_calc_ut jd=%f, i=%d, bodies[i]=%d, lon=%f, lat=%f, dist=%f, swe_degnorm=%f", jd_ut, i,bodies[i], xret[0], xret[1], xret[3], swe_degnorm(xret[0])));

        st->results[i][0] = (double)bodies[i];      /* idplanet 0 … 9           */
        st->results[i][1] = swe_degnorm(xret[0]);  /* longitude          */
	if (bodies[i] == SE_MOON) moon = xret[0]; 
	if (bodies[i] == SE_SUN) sun = xret[0]; 
        st->results[i][2] = xret[1];              /* latitude            */
        st->results[i][3] = xret[3];              /* distance speed (AU)       */
    }

/*   for (i = 0; i < 11  ; i++) { // Astéroides 40–48
    
        double xret[6];
        if (swe_calc_ut(jd_ut, i, iflag, xret, serr) == ERR)
            ereport(ERROR, (errmsg("we_calc_ut error: %s", serr)));
        st->results[i][0] = (double)asteroids[i];      // idplanet 21 .. 29           
        st->results[i][1] = swe_degnorm(xret[0]);  // longitude          
        st->results[i][2] = xret[1];              // latitude            
        st->results[i][3] = xret[3];              // distance speed (AU)       
    }
*/

    /* 2) Ascendant / MC ---------------------------------------------- */
    double cusps[13];      /* non utilisé ici, mais requis       */
    double ascmc[10];
    double cusp_speed[13];
    double ascmc_speed[10];
    if (swe_houses_ex2(jd_ut, iflag, lat_deg, lon_deg, 'R', cusps, ascmc, cusp_speed, ascmc_speed, serr) == ERR) {
        ereport(INFO, (errmsg("swe_houses_ex failed lat=%f, lon=%f", lat_deg, lon_deg)));
	//ascmc[SE_ASC] = 0;
	//ascmc[SE_MC] = 0; 
    }




    /* Ascendant, MC, leurs oppositions */
    double asc = swe_degnorm(ascmc[SE_ASC]);
    double mc  = swe_degnorm(ascmc[SE_MC]);
    double dsc = swe_degnorm(asc + 180.0);
    double ic  = swe_degnorm(mc  + 180.0);

    int base = BODIES_COUNT;// + 30 car on a les astéroides 40–48
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


    //Part du monde
    st->results[base+4][0] = ID_PART;
    st->results[base+4][1] = swe_degnorm( moon -sun);
    st->results[base+4][2] = 0.0;
    st->results[base+4][3] = 0.0;


    //Bacchus
    iflag = SEFLG_SWIEPH | SEFLG_SPEED;   // vous pouvez ajouter  
                                            // SEFLG_NONUT, etc.    
    double lon_bacchus = lambda_geo_fast(jd_ut, bacchus_helio(jd_ut),  RADIUS_BACCHUS);
    double lon_apollon = lambda_geo_fast(jd_ut, apollon_helio(jd_ut),  RADIUS_APOLLON);
    double lon_proserpine = lambda_geo_fast(jd_ut, proserpine_helio(jd_ut),  RADIUS_PROSERPINE);

    //int asteroids2[3] = {25, 26, 27};// Bacchus, apollon, proserpine


    base = 24;
        /* bacchus */
    st->results[base+1][0] = 25;
    st->results[base+1][1] = lon_bacchus;
    st->results[base+1][2] = 0.0;
    st->results[base+1][3] = 0.0;

           /* apollon */
    st->results[base+2][0] = 26;
    st->results[base+2][1] = lon_apollon;
    st->results[base+2][2] = 0.0;
    st->results[base+2][3] = 0.0;


    /* proserpine */

    st->results[base+3][0] = 27;
    st->results[base+3][1] = lon_proserpine;
    st->results[base+3][2] = 0.0;
    st->results[base+3][3] = 0.0;

    //itère sur st->results[i][1} pour tous les i et affiche un ereport
    /*for (i = 0; i < NB_RESULTS; i++) {
	ereport(INFO, errmsg("LOG compute_positions: %d: lon=%.8f lat=%.8f dist=%.8f",
			     (int)st->results[i][0], st->results[i][1],
			     st->results[i][2], st->results[i][3]));
    }*/

	

}

/* ------------------------------------------------------------------ */
/*  Fonction SRF visible depuis SQL                                   */
/* ------------------------------------------------------------------ */
PG_FUNCTION_INFO_V1(sw_planet_positions);

Datum sw_planet_positions(PG_FUNCTION_ARGS) {
    FuncCallContext   *funcctx;
    calc_state        *st;

    /* première entrée ------------------------------------------------ */
    if (SRF_IS_FIRSTCALL()) {
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

        swe_set_ephe_path(SE_EPHE_PATH);   /* path paquet Debian */

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

    if (st->current < st->nrows) {
        Datum       values[5];
        bool        nulls[5] = {false,false,false,false,false};

        values[0] = TimestampGetDatum(st->ts);
        values[1] = Int32GetDatum((int32) st->results[st->current][0]);
        values[2] = Float8GetDatum(st->results[st->current][1]);
        values[3] = Float8GetDatum(st->results[st->current][2]);
        values[4] = Float8GetDatum(st->results[st->current][3]);

        HeapTuple   tuple = heap_form_tuple(funcctx->tuple_desc, values, nulls);
        st->current++;

        SRF_RETURN_NEXT(funcctx, HeapTupleGetDatum(tuple));
    } else {
        SRF_RETURN_DONE(funcctx);
    }
}

