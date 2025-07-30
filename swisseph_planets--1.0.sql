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
            (2, 'Gemaux'),
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
