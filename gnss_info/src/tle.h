/*
 * The RTKLIB software package is distributed under the following BSD 2-clause
 * license (http://opensource.org/licenses/BSD-2-Clause) and additional two
 * exclusive clauses. Users are permitted to develop, produce or sell their own
 * non-commercial or commercial products utilizing, linking or including RTKLIB as
 * long as they comply with the license.
 *
 *           Copyright (c) 2007-2013, T. Takasu, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * - The software package includes some companion executive binaries or shared
 *   libraries necessary to execute APs on Windows. These licenses succeed to the
 *   original ones of these software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// This file is an excerpt from rtklib.

#pragma once

/*------------------------------------------------------------------------------
* tle.c: NORAD TLE (two line element) functions
*
*          Copyright (C) 2012-2013 by T.TAKASU, All rights reserved.
*
* references:
*     [1] F.R.Hoots and R.L.Roehrich, Spacetrack report No.3, Models for
*         propagation of NORAD element sets, December 1980
*     [2] D.A.Vallado, P.Crawford, R.Hujsak and T.S.Kelso, Revisiting
*         Spacetrack Report #3, AIAA 2006-6753, 2006
*     [3] CelesTrak (http://www.celestrak.com)
*
* version : $Revision:$ $Date:$
* history : 2012/11/01 1.0  new
*           2013/01/25 1.1  fix bug on binary search
*           2014/08/26 1.2  fix bug on tle_pos() to get tle by satid or desig
*-----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} gtime_t;

typedef struct {        /* earth rotation parameter data type */
    double mjd;         /* mjd (days) */
    double xp,yp;       /* pole offset (rad) */
    double xpr,ypr;     /* pole offset rate (rad/day) */
    double ut1_utc;     /* ut1-utc (s) */
    double lod;         /* length of day (s/day) */
} erpd_t;

typedef struct {        /* earth rotation parameter type */
    int n,nmax;         /* number and max number of data */
    erpd_t *data;       /* earth rotation parameter data */
} erp_t;

typedef struct {        /* norad two line element data type */
    char name [32];     /* common name */
    char alias[32];     /* alias name */
    char satno[16];     /* satellilte catalog number */
    char satclass;      /* classification */
    char desig[16];     /* international designator */
    gtime_t epoch;      /* element set epoch (UTC) */
    double ndot;        /* 1st derivative of mean motion */
    double nddot;       /* 2st derivative of mean motion */
    double bstar;       /* B* drag term */
    int etype;          /* element set type */
    int eleno;          /* element number */
    double inc;         /* orbit inclination (deg) */
    double OMG;         /* right ascension of ascending node (deg) */
    double ecc;         /* eccentricity */
    double omg;         /* argument of perigee (deg) */
    double M;           /* mean anomaly (deg) */
    double n;           /* mean motion (rev/day) */
    int rev;            /* revolution number at epoch */
} tled_t;

typedef struct {        /* norad two line element type */
    int n,nmax;         /* number/max number of two line element data */
    tled_t *data;       /* norad two line element data */
} tle_t;

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t utc2gpst(gtime_t t);

/* read TLE file ---------------------------------------------------------------
* read NORAD TLE (two line element) data file (ref [2],[3])
* args   : char   *file     I   NORAD TLE data file
*          tle_t  *tle      O   TLE data
* return : status (1:ok,0:error)
* notes  : before calling the function, the TLE data should be initialized.
*          the file should be in a two line (only TLE) or three line (satellite
*          name + TLE) format.
*          the characters after # in a line are treated as comments.
*-----------------------------------------------------------------------------*/
int tle_read(const char *file, tle_t *tle);

/* satellite position and velocity with TLE data -------------------------------
* compute satellite position and velocity in ECEF with TLE data
* args   : gtime_t time     I   time (GPST)
*          char   *name     I   satellite name           ("": not specified)
*          char   *satno    I   satellite catalog number ("": not specified)
*          char   *desig    I   international designaor  ("": not specified)
*          tle_t  *tle      I   TLE data
*          erp_t  *erp      I   EOP data (NULL: not used)
*          double *rs       O   sat position/velocity {x,y,z,vx,vy,vz} (m,m/s)
* return : status (1:ok,0:error)
* notes  : the coordinates of the position and velocity are ECEF (ITRF)
*          if erp == NULL, polar motion and ut1-utc are neglected
*-----------------------------------------------------------------------------*/
int tle_pos(gtime_t time, const char *name, const char *satno,
                   const char *desig, const tle_t *tle, const erp_t *erp,
                   double *rs);

#ifdef __cplusplus
}
#endif