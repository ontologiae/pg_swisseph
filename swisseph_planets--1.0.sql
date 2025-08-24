-- Extension : swisseph_planets 1.0
CREATE FUNCTION sw_planet_positions(
          ts        timestamptz,
          latitude  double precision,
          longitude double precision)
RETURNS TABLE (
          ts        timestamptz,
          idplanet  integer,
          lon       double precision,
          lat       double precision,
          distspeed double precision,
	  dist 	    double precision)
AS 'MODULE_PATHNAME', 'sw_planet_positions'
LANGUAGE C
STRICT
PARALLEL SAFE;

CREATE OR REPLACE FUNCTION get_zodiac_sign(degrees FLOAT)
RETURNS TABLE(lib TEXT, deg FLOAT) AS $$
BEGIN
    RETURN QUERY
    WITH signes(id, lib) AS (
        VALUES
            (0, 'Bélier'),
            (1, 'Taureau'),
            (2, 'Gémeaux'),
            (3, 'Cancer'),
            (4, 'Lion'),
            (5, 'Vierge'),
            (6, 'Balance'),
            (7, 'Scorpion'),
            (8, 'Sagitaire'),
            (9, 'Capricorne'),
            (10, 'Verseau'),
            (11, 'Poisson')
    )
    SELECT s.lib, degrees -  ( (floor(degrees / 30)) * 30 + (degrees::int % 30) ) + (degrees::int % 30)  AS deg
    FROM signes s
    WHERE s.id = floor(degrees / 30);
END;
$$ LANGUAGE plpgsql;


Create type sensi as Enum('Direct', 'Retrograde', 'Stationnary');

CREATE OR REPLACE FUNCTION get_sensi(speed FLOAT)
RETURNS sensi AS $$
BEGIN
	If trunc(speed::numeric,5) = 0 Then Return 'Stationnary'; End If;	
	IF speed > 0 THEN RETURN 'Direct'; Else Return 'Retrograde'; END IF;
	--We'll see next time to evaluates when it is considered stationnary
END;
$$ LANGUAGE plpgsql;

/*
Mercury 	300
Venus 	300
Mars 	90
Jupiter 	60
Saturn 	60
Chiron 	20
Uranus 	20
Neptune 	10
Pluto 	10*/


CREATE OR REPLACE FUNCTION get_body_name(bodyid INTEGER)
RETURNS TABLE(lib TEXT) AS $$
BEGIN
    RETURN QUERY
    WITH bodies(id, lib) AS (
        VALUES
	(1, 'Moon'),
	(0,'Sun'),
	(2,'Mercury'),       
	(3,'Venus'),       
	(4,'Mars'),       
	(5,'Jupiter'),       
	(6,'Saturn'),       
	(7,'Uranus'),       
	(8,'Neptune'),       
	(9,'Pluto'),       
	(10,'Mean_node'),      
	(11,'True_node'),
	(12,'South Node'),      
	(13,'Oscu_apog'),    
	(14,'Earth'),      
	(15,'Chiron'),      
	(16,'Pholus'),      
	(17,'Ceres'),      
	(18,'Pallas'),      
	(19,'Juno'),      
	(20,'Vesta'),
	(31,'Eris'),
	(32,'Sedna'),
	(33,'Haumea'),
	(34,'Makemake'),
	(35,'Quaoar'),
	(25,'Bacchus'),
	(26,'Apollon'),
	(27,'Proserpine (Kore)'),
	(10001,'Ascendant'),
	(10002,'Descendant'),
	(10003,'Medium Coeli'),
	(10004,'Imum Coeli'),
	(10005,'Part du monde'),
	(10006,'Part de fortune'),
	(10022,'House 2'),
	(10023,'House 3'),
	(10024,'House 4'),
	(10025,'House 5'),
	(10026,'House 6'),
	(10027,'House 7'),
	(10028,'House 8'),
	(10029,'House 9'),
	(10030,'House 10'),
	(10031,'House 11'),
	(10032,'House 12'),
	(30000,'Varuna'),
	(17066,'Nessus'),
	(90482+10000,'Orcus'),--SE_AST_OFFSET + asteroid international code
	(10000+29748,'Ixion'),
	(10000+136199,'Eris'),
	(10000+90377,'Sedna'),
	(10000+136108,'Haumea'),
	(10000+136472,'Makemake'),
	(10000+50000,'Quaoar'),
	(10000+28978,'Ixion'),	
	(10000+16,'Psyché'),
	(10000+1221,'Amor'),
	(10000+433,'Eros'),
	(10000+15760,'Isis'),
	(55,'Vulcan'),
	(47,'Poseidon')
)
SELECT s.lib
FROM bodies s
WHERE s.id = bodyid;
END;
$$ LANGUAGE plpgsql;


-- write a  PL function to convert decimal to sexagesimal
CREATE OR REPLACE FUNCTION decimal_to_sexagesimal(degrees double precision)
RETURNS TABLE(deg INTEGER, min INTEGER, sec FLOAT) AS $$
BEGIN
    RETURN QUERY
    SELECT 
	floor(degrees)::int AS deg,
	floor((degrees - floor(degrees)) * 60)::int AS min,
	(degrees - floor(degrees) - (floor((degrees - floor(degrees)) * 60) / 60)) * 3600 AS sec;
END;
$$ LANGUAGE plpgsql;
