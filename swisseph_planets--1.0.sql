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
          dist      double precision)
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



CREATE OR REPLACE FUNCTION get_body_name(bodyid INTEGER)
RETURNS TABLE(lib TEXT) AS $$
BEGIN
    RETURN QUERY
    WITH bodies(id, lib) AS (
        VALUES
	(11, 'Poisson'),
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
	(12,'Mean_apog'),      
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
	(30000,'Varuna'),
	(55,'Vulcan'),
	(47,'Poseidon'),
	(146199,'Eris')
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
