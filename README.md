For Debian/Ubuntu :


sudo apt-get install libswe2.0 libswe-dev swe-data swe-extra-data swe-basic-data swe-standard-data

Building :

make
sudo make install        

In your database :

CREATE EXTENSION swisseph_planets;

Example : 

```sql
SELECT idplanet, (get_zodiac_sign(lon)).lib, (get_zodiac_sign(lon)).deg FROM   sw_planet_positions('2025-07-27 12:00:00+02'::timestamp, 47.22,-1.58);
INFO:  LOG swe_utc_to_jd : 2460884.000003
 idplanet |    lib     |        deg         
----------+------------+--------------------
        0 | Lion       |  4.719286963086262
        1 | Vierge     |   8.72453495781616
        2 | Lion       | 12.247413429433408
        3 | Gemaux     | 25.772977381299924
        4 | Vierge     | 23.541208553550888
        5 | Cancer     |   10.7423866052183
        6 | Bélier     | 1.7622344708520048
        7 | Gemaux     | 0.7830286509269158
        8 | Bélier     |  2.040676852514073
        9 | Verseau    |  2.537321723661819
       10 | Poisson    | 20.510583146444333
       11 | Poisson    | 19.014509597275264
       12 | Scorpion   | 13.640307301696055
       13 | Scorpion   | 24.870845385805524
       14 | Bélier     |                  0
       15 | Bélier     | 27.158360506281117
       16 | Capricorne |  9.380016514938632
       17 | Bélier     |  16.11664294153043
       18 | Verseau    | 18.554493284340992
       19 | Scorpion   | 16.920215913464773
       20 | Scorpion   | 11.973520764426212
    10001 | Balance    | 25.045161213798877
    10002 | Bélier     | 25.045161213798906
    10003 | Lion       | 1.6003619357428107
    10004 | Verseau    | 1.6003619357427965
       44 | Scorpion   |  5.671649726713696
       45 | Gemaux     |  5.317503955528153
       46 | Lion       |  4.223562648126844
       47 | Scorpion   | 15.653614442338636
       48 | Vierge     |  3.954320658520146
        0 | Bélier     |                  0
        0 | Bélier     |                  0
        0 | Bélier     |                  0
        0 | Bélier     |                  0
(34 rows)
```
