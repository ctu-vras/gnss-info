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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "tle.h"

#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */
#define PI          3.1415926535897932  /* pi */

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   25                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   62                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */
#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATSBS+NSATLEO)

#define MAXLEAPS    64                  /* max number of leap seconds table */
static double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
    {2017,1,1,0,0,0,-18},
    {2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1},
    {0}
};

void trace(int level, const char *format, ...)
{
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t timeadd(gtime_t t, double sec)
{
    double tt;

    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

    if (year<1970||2099<year||mon<1||12<mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2utc(gtime_t t)
{
    gtime_t tu;
    int i;

    for (i=0;leaps[i][0]>0;i++) {
        tu=timeadd(t,leaps[i][6]);
        if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
    }
    return t;
}

gtime_t utc2gpst(gtime_t t)
{
    int i;

    for (i=0;leaps[i][0]>0;i++) {
        if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
    }
    return t;
}

/* get earth rotation parameter values -----------------------------------------
* get earth rotation parameter values
* args   : erp_t  *erp        I   earth rotation parameters
*          gtime_t time       I   time (gpst)
*          double *erpv       O   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int geterp(const erp_t *erp, gtime_t time, double *erpv)
{
    const double ep[]={2000,1,1,12,0,0};
    double mjd,day,a;
    int i,j,k;

    trace(4,"geterp:\n");

    if (erp->n<=0) return 0;

    mjd=51544.5+(timediff(gpst2utc(time),epoch2time(ep)))/86400.0;

    if (mjd<=erp->data[0].mjd) {
        day=mjd-erp->data[0].mjd;
        erpv[0]=erp->data[0].xp     +erp->data[0].xpr*day;
        erpv[1]=erp->data[0].yp     +erp->data[0].ypr*day;
        erpv[2]=erp->data[0].ut1_utc-erp->data[0].lod*day;
        erpv[3]=erp->data[0].lod;
        return 1;
    }
    if (mjd>=erp->data[erp->n-1].mjd) {
        day=mjd-erp->data[erp->n-1].mjd;
        erpv[0]=erp->data[erp->n-1].xp     +erp->data[erp->n-1].xpr*day;
        erpv[1]=erp->data[erp->n-1].yp     +erp->data[erp->n-1].ypr*day;
        erpv[2]=erp->data[erp->n-1].ut1_utc-erp->data[erp->n-1].lod*day;
        erpv[3]=erp->data[erp->n-1].lod;
        return 1;
    }
    for (j=0,k=erp->n-1;j<k-1;) {
        i=(j+k)/2;
        if (mjd<erp->data[i].mjd) k=i; else j=i;
    }
    if (erp->data[j].mjd==erp->data[j+1].mjd) {
        a=0.5;
    }
    else {
        a=(mjd-erp->data[j].mjd)/(erp->data[j+1].mjd-erp->data[j].mjd);
    }
    erpv[0]=(1.0-a)*erp->data[j].xp     +a*erp->data[j+1].xp;
    erpv[1]=(1.0-a)*erp->data[j].yp     +a*erp->data[j+1].yp;
    erpv[2]=(1.0-a)*erp->data[j].ut1_utc+a*erp->data[j+1].ut1_utc;
    erpv[3]=(1.0-a)*erp->data[j].lod    +a*erp->data[j+1].lod;
    return 1;
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
void time2epoch(gtime_t t, double *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;

    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t *day)
{
    double ep[6],sec;
    time2epoch(time,ep);
    sec=ep[3]*3600.0+ep[4]*60.0+ep[5];
    ep[3]=ep[4]=ep[5]=0.0;
    *day=epoch2time(ep);
    return sec;
}

/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
double utc2gmst(gtime_t t, double ut1_utc)
{
    const double ep2000[]={2000,1,1,12,0,0};
    gtime_t tut,tut0;
    double ut,t1,t2,t3,gmst0,gmst;

    tut=timeadd(t,ut1_utc);
    ut=time2sec(tut,&tut0);
    t1=timediff(tut0,epoch2time(ep2000))/86400.0/36525.0;
    t2=t1*t1; t3=t2*t1;
    gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
    gmst=gmst0+1.002737909350795*ut;

    return fmod(gmst,86400.0)*PI/43200.0; /* 0 <= gmst <= 2*PI */
}

/* multiply matrix -----------------------------------------------------------*/
void matmul(const char *tr, int n, int k, int m, double alpha,
                   const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

    for (i=0;i<n;i++) for (j=0;j<k;j++) {
        d=0.0;
        switch (f) {
            case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
            case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
            case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
            case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
        }
        if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
    }
}

/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
double str2num(const char *s, int i, int n)
{
    double value;
    char str[256],*p=str;

    if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<n) return 0.0;
    for (s+=i;*s&&--n>=0;s++) *p++=*s=='d'||*s=='D'?'E':*s; *p='\0';
    return sscanf(str,"%lf",&value)==1?value:0.0;
}

/* SGP4 model propagator by STR#3 (ref [1] sec.6,11) -------------------------*/

#define DE2RA       0.174532925E-1
#define E6A         1.E-6
#define TOTHRD      0.66666667
#define TWOPI       6.2831853
#define XJ3         -0.253881E-5
#define XKE         0.743669161E-1
#define XKMPER      6378.135
#define XMNPDA      1440.0
#define AE          1.0
#define CK2         5.413080E-4         /* = 0.5*XJ2*AE*AE */
#define CK4         0.62098875E-6       /* = -0.375*XJ4*AE*AE*AE*AE */
#define QOMS2T      1.88027916E-9       /* = pow((QO-SO)*AE/XKMPER,4.0) */
#define S           1.01222928          /* = AE*(1.0+SO/XKMPER) */

static void SGP4_STR3(double tsince, const tled_t *data, double *rs)
{
    double xnodeo,omegao,xmo,eo,xincl,xno,xndt2o,xndd6o,bstar;
    double a1,cosio,theta2,x3thm1,eosq,betao2,betao,del1,ao,delo,xnodp,aodp,s4;
    double qoms24,perige,pinvsq,tsi,eta,etasq,eeta,psisq,coef,coef1,c1,c2,c3,c4;
    double c5,sinio,a3ovk2,x1mth2,theta4,xmdot,x1m5th,omgdot,xhdot1,xnodot;
    double omgcof,xmcof,xnodcf,t2cof,xlcof,aycof,delmo,sinmo,x7thm1,c1sq,d2,d3;
    double d4,t3cof,t4cof,t5cof,xmdf,omgadf,xnoddf,omega,xmp,tsq,xnode,delomg;
    double delm,tcube,tfour,a,e,xl,beta,xn,axn,xll,aynl,xlt,ayn,capu,sinepw;
    double cosepw,epw,ecose,esine,elsq,pl,r,rdot,rfdot,betal,cosu,sinu,u,sin2u;
    double cos2u,rk,uk,xnodek,xinck,rdotk,rfdotk,sinuk,cosuk,sinik,cosik,sinnok;
    double cosnok,xmx,xmy,ux,uy,uz,vx,vy,vz,x,y,z,xdot,ydot,zdot;
    double temp,temp1,temp2,temp3,temp4,temp5,temp6,tempa,tempe,templ;
    int i,isimp;

    xnodeo=data->OMG*DE2RA;
    omegao=data->omg*DE2RA;
    xmo=data->M*DE2RA;
    xincl=data->inc*DE2RA;
    temp=TWOPI/XMNPDA/XMNPDA;
    xno=data->n*temp*XMNPDA;
    xndt2o=data->ndot*temp;
    xndd6o=data->nddot*temp/XMNPDA;
    bstar=data->bstar/AE;
    eo=data->ecc;
    /*
    * recover original mean motion (xnodp) and semimajor axis (aodp)
    * from input elements
    */
    a1=pow(XKE/xno,TOTHRD);
    cosio=cos(xincl);
    theta2=cosio*cosio;
    x3thm1=3.0*theta2-1.0;
    eosq=eo*eo;
    betao2=1.0-eosq;
    betao=sqrt(betao2);
    del1=1.5*CK2*x3thm1/(a1*a1*betao*betao2);
    ao=a1*(1.0-del1*(0.5*TOTHRD+del1*(1.0+134.0/81.0*del1)));
    delo=1.5*CK2*x3thm1/(ao*ao*betao*betao2);
    xnodp=xno/(1.0+delo);
    aodp=ao/(1.0-delo);
    /*
    * initialization
    * for perigee less than 220 kilometers, the isimp flag is set and
    * the equations are truncated to linear variation in sqrt a and
    * quadratic variation in mean anomaly. also, the c3 term, the
    * delta omega term, and the delta m term are dropped.
    */
    isimp=0;
    if ((aodp*(1.0-eo)/AE)<(220.0/XKMPER+AE)) isimp=1;

    /* for perigee below 156 km, the values of s and qoms2t are altered */
    s4=S;
    qoms24=QOMS2T;
    perige=(aodp*(1.0-eo)-AE)*XKMPER;
    if (perige<156.0) {
        s4=perige-78.0;
        if (perige<=98.0) s4=20.0;
        qoms24=pow((120.0-s4)*AE/XKMPER,4.0);
        s4=s4/XKMPER+AE;
    }
    pinvsq=1.0/(aodp*aodp*betao2*betao2);
    tsi=1.0/(aodp-s4);
    eta=aodp*eo*tsi;
    etasq=eta*eta;
    eeta=eo*eta;
    psisq=fabs(1.0-etasq);
    coef=qoms24*pow(tsi,4.0);
    coef1=coef/pow(psisq,3.5);
    c2=coef1*xnodp*(aodp*(1.0+1.5*etasq+eeta*(4.0+etasq))+0.75*
        CK2*tsi/psisq*x3thm1*(8.0+3.0*etasq*(8.0+etasq)));
    c1=bstar*c2;
    sinio=sin(xincl);
    a3ovk2=-XJ3/CK2*pow(AE,3.0);
    c3=coef*tsi*a3ovk2*xnodp*AE*sinio/eo;
    x1mth2=1.0-theta2;
    c4=2.0*xnodp*coef1*aodp*betao2*(eta*
        (2.0+0.5*etasq)+eo*(0.5+2.0*etasq)-2.0*CK2*tsi/
        (aodp*psisq)*(-3.0*x3thm1*(1.0-2.0*eeta+etasq*
        (1.5-0.5*eeta))+0.75*x1mth2*(2.0*etasq-eeta*
        (1.0+etasq))*cos(2.0*omegao)));
    c5=2.0*coef1*aodp*betao2*(1.0+2.75*(etasq+eeta)+eeta*etasq);
    theta4=theta2*theta2;
    temp1=3.0*CK2*pinvsq*xnodp;
    temp2=temp1*CK2*pinvsq;
    temp3=1.25*CK4*pinvsq*pinvsq*xnodp;
    xmdot=xnodp+0.5*temp1*betao*x3thm1+0.0625*temp2*betao*
        (13.0-78.0*theta2+137.0*theta4);
    x1m5th=1.0-5.0*theta2;
    omgdot=-0.5*temp1*x1m5th+0.0625*temp2*(7.0-114.0*theta2+
        395.0*theta4)+temp3*(3.0-36.0*theta2+49.0*theta4);
    xhdot1=-temp1*cosio;
    xnodot=xhdot1+(0.5*temp2*(4.0-19.0*theta2)+2.0*temp3*(3.0-
        7.0*theta2))*cosio;
    omgcof=bstar*c3*cos(omegao);
    xmcof=-TOTHRD*coef*bstar*AE/eeta;
    xnodcf=3.5*betao2*xhdot1*c1;
    t2cof=1.5*c1;
    xlcof=0.125*a3ovk2*sinio*(3.0+5.0*cosio)/(1.0+cosio);
    aycof=0.25*a3ovk2*sinio;
    delmo=pow(1.0+eta*cos(xmo),3.0);
    sinmo=sin(xmo);
    x7thm1=7.0*theta2-1.0;

    if (isimp!=1) {
        c1sq=c1*c1;
        d2=4.0*aodp*tsi*c1sq;
        temp=d2*tsi*c1/3.0;
        d3=(17.0*aodp+s4)*temp;
        d4=0.5*temp*aodp*tsi*(221.0*aodp+31.0*s4)*c1;
        t3cof=d2+2.0*c1sq;
        t4cof=0.25*(3.0*d3+c1*(12.0*d2+10.0*c1sq));
        t5cof=0.2*(3.0*d4+12.0*c1*d3+6.0*d2*d2+15.0*c1sq*(2.0*d2+c1sq));
    }
    else {
        d2=d3=d4=t3cof=t4cof=t5cof=0.0;
    }
    /* update for secular gravity and atmospheric drag */
    xmdf=xmo+xmdot*tsince;
    omgadf=omegao+omgdot*tsince;
    xnoddf=xnodeo+xnodot*tsince;
    omega=omgadf;
    xmp=xmdf;
    tsq=tsince*tsince;
    xnode=xnoddf+xnodcf*tsq;
    tempa=1.0-c1*tsince;
    tempe=bstar*c4*tsince;
    templ=t2cof*tsq;
    if (isimp==1) {
        delomg=omgcof*tsince;
        delm=xmcof*(pow(1.0+eta*cos(xmdf),3.0)-delmo);
        temp=delomg+delm;
        xmp=xmdf+temp;
        omega=omgadf-temp;
        tcube=tsq*tsince;
        tfour=tsince*tcube;
        tempa=tempa-d2*tsq-d3*tcube-d4*tfour;
        tempe=tempe+bstar*c5*(sin(xmp)-sinmo);
        templ=templ+t3cof*tcube+tfour*(t4cof+tsince*t5cof);
    }
    a=aodp*pow(tempa,2.0);
    e=eo-tempe;
    xl=xmp+omega+xnode+xnodp*templ;
    beta=sqrt(1.0-e*e);
    xn=XKE/pow(a,1.5);

    /* long period periodics */
    axn=e*cos(omega);
    temp=1.0/(a*beta*beta);
    xll=temp*xlcof*axn;
    aynl=temp*aycof;
    xlt=xl+xll;
    ayn=e*sin(omega)+aynl;

    /* solve keplers equation */
    capu=fmod(xlt-xnode,TWOPI);
    temp2=capu;
    for (i=0;i<10;i++) {
        sinepw=sin(temp2);
        cosepw=cos(temp2);
        temp3=axn*sinepw;
        temp4=ayn*cosepw;
        temp5=axn*cosepw;
        temp6=ayn*sinepw;
        epw=(capu-temp4+temp3-temp2)/(1.0-temp5-temp6)+temp2;
        if (fabs(epw-temp2)<=E6A) break;
        temp2=epw;
    }
    /* short period preliminary quantities */
    ecose=temp5+temp6;
    esine=temp3-temp4;
    elsq=axn*axn+ayn*ayn;
    temp=1.0-elsq;
    pl=a*temp;
    r=a*(1.0-ecose);
    temp1=1.0/r;
    rdot=XKE*sqrt(a)*esine*temp1;
    rfdot=XKE*sqrt(pl)*temp1;
    temp2=a*temp1;
    betal=sqrt(temp);
    temp3=1.0/(1.0+betal);
    cosu=temp2*(cosepw-axn+ayn*esine*temp3);
    sinu=temp2*(sinepw-ayn-axn*esine*temp3);
    u=atan2(sinu,cosu);
    sin2u=2.0*sinu*cosu;
    cos2u=2.0*cosu*cosu-1.0;
    temp=1.0/pl;
    temp1=CK2*temp;
    temp2=temp1*temp;

    /* update for short periodics */
    rk=r*(1.0-1.5*temp2*betal*x3thm1)+0.5*temp1*x1mth2*cos2u;
    uk=u-0.25*temp2*x7thm1*sin2u;
    xnodek=xnode+1.5*temp2*cosio*sin2u;
    xinck=xincl+1.5*temp2*cosio*sinio*cos2u;
    rdotk=rdot-xn*temp1*x1mth2*sin2u;
    rfdotk=rfdot+xn*temp1*(x1mth2*cos2u+1.5*x3thm1);

    /* orientation vectors */
    sinuk=sin(uk);
    cosuk=cos(uk);
    sinik=sin(xinck);
    cosik=cos(xinck);
    sinnok=sin(xnodek);
    cosnok=cos(xnodek);
    xmx=-sinnok*cosik;
    xmy=cosnok*cosik;
    ux=xmx*sinuk+cosnok*cosuk;
    uy=xmy*sinuk+sinnok*cosuk;
    uz=sinik*sinuk;
    vx=xmx*cosuk-cosnok*sinuk;
    vy=xmy*cosuk-sinnok*sinuk;
    vz=sinik*cosuk;

    /* position and velocity */
    x=rk*ux;
    y=rk*uy;
    z=rk*uz;
    xdot=rdotk*ux+rfdotk*vx;
    ydot=rdotk*uy+rfdotk*vy;
    zdot=rdotk*uz+rfdotk*vz;

    rs[0]=x*XKMPER/AE*1E3; /* (m) */
    rs[1]=y*XKMPER/AE*1E3;
    rs[2]=z*XKMPER/AE*1E3;
    rs[3]=xdot*XKMPER/AE*XMNPDA/86400.0*1E3; /* (m/s) */
    rs[4]=ydot*XKMPER/AE*XMNPDA/86400.0*1E3;
    rs[5]=zdot*XKMPER/AE*XMNPDA/86400.0*1E3;
}
/* drop spaces at string tail ------------------------------------------------*/
static void chop(char *buff)
{
    int i;
    for (i=strlen(buff)-1;i>=0;i--) {
        if (buff[i]==' '||buff[i]=='\r'||buff[i]=='\n') buff[i]='\0';
        else break;
    }
}
/* test TLE line checksum ----------------------------------------------------*/
static int checksum(const char *buff)
{
    int i,cs=0;

    if (strlen(buff)<69) return 0;

    for (i=0;i<68;i++) {
        if ('0'<=buff[i]&&buff[i]<='9') cs+=(int)(buff[i]-'0');
        else if (buff[i]=='-') cs+=1;
    }
    return (int)(buff[68]-'0')==cs%10;
}
/* decode TLE line 1 ---------------------------------------------------------*/
static int decode_line1(const char *buff, tled_t *data)
{
    double year,doy,nddot,exp1,bstar,exp2,ep[6]={2000,1,1};

    strncpy(data->satno,buff+2,5);       /* satellite number */
    data->satno[5]='\0';
    chop(data->satno);

    data->satclass=buff[7];              /* satellite classification */
    strncpy(data->desig,buff+9,8);       /* international designator */
    data->desig[8]='\0';
    chop(data->desig);

    year      =str2num(buff,18, 2);      /* epoch year */
    doy       =str2num(buff,20,12);      /* epoch day of year */
    data->ndot=str2num(buff,33,10);      /* 1st time derivative of n */
    nddot     =str2num(buff,44, 6);      /* 2nd time derivative of n */
    exp1      =str2num(buff,50, 2);
    bstar     =str2num(buff,53, 6);      /* Bstar drag term */
    exp2      =str2num(buff,59, 2);
    data->etype=(int)str2num(buff,62,1); /* ephemeris type */
    data->eleno=(int)str2num(buff,64,4); /* ephemeris number */
    data->nddot=nddot*1E-5*pow(10.0,exp1);
    data->bstar=bstar*1E-5*pow(10.0,exp2);

    ep[0]=year+(year<57.0?2000.0:1900.0);
    data->epoch=timeadd(epoch2time(ep),(doy-1.0)*86400.0);

    data->inc=data->OMG=data->ecc=data->omg=data->M=data->n=0.0;
    data->rev=0;
    return 1;
}
/* decode TLE line 2 ---------------------------------------------------------*/
static int decode_line2(const char *buff, tled_t *data)
{
    char satno[16];

    strncpy(satno,buff+2,5);             /* satellite number */
    satno[5]='\0';
    chop(satno);

    data->inc=str2num(buff, 8, 8);       /* inclination (deg) */
    data->OMG=str2num(buff,17, 8);       /* RAAN (deg) */
    data->ecc=str2num(buff,26, 7)*1E-7;  /* eccentricity */
    data->omg=str2num(buff,34, 8);       /* argument of perigee (deg) */
    data->M  =str2num(buff,43, 8);       /* mean anomaly (deg) */
    data->n  =str2num(buff,52,11);       /* mean motion (rev/day) */
    data->rev=(int)str2num(buff,63,5);   /* revolution number */

    if (strcmp(satno,data->satno)) {
        trace(2,"tle satno mismatch: %s %s\n",data->satno,satno);
        return 0;
    }
    if (data->n<=0.0||data->ecc<0.0) {
        trace(2,"tle data error: %s\n",satno);
        return 0;
    }
    return 1;
}
/* add TLE data --------------------------------------------------------------*/
static int add_data(tle_t *tle, const tled_t *data)
{
    tled_t *tle_data;

    if (tle->n>=tle->nmax) {
        tle->nmax=tle->nmax<=0?1024:tle->nmax*2;

        if (!(tle_data=(tled_t *)realloc(tle->data,sizeof(tled_t)*tle->nmax))) {
            trace(1,"tle malloc error\n");
            free(tle->data); tle->data=NULL; tle->n=tle->nmax=0;
            return 0;
        }
        tle->data=tle_data;
    }
    tle->data[tle->n++]=*data;
    return 1;
}
/* compare TLE data by satellite name ----------------------------------------*/
static int cmp_tle_data(const void *p1, const void *p2)
{
    const tled_t *q1=(const tled_t *)p1,*q2=(const tled_t *)p2;
    return strcmp(q1->name,q2->name);
}

int tle_read(const char *file, tle_t *tle)
{
    FILE *fp;
    tled_t data={{0}};
    char *p,buff[256];
    int line=0;

    if (!(fp=fopen(file,"r"))) {
        trace(2,"tle file open error: %s\n",file);
        return 0;
    }
    while (fgets(buff,sizeof(buff),fp)) {

        /* delete comments */
        if ((p=strchr(buff,'#'))) *p='\0';
        chop(buff);

        if (buff[0]=='1'&&checksum(buff)) {

            /* decode TLE line 1 */
            if (decode_line1(buff,&data)) line=1;
        }
        else if (line==1&&buff[0]=='2'&&checksum(buff)) {

            /* decode TLE line 2 */
            if (!decode_line2(buff,&data)) continue;

            /* add TLE data */
            if (!add_data(tle,&data)) {
                fclose(fp);
                return 0;
            }
            data.name[0]='\0';
            data.alias[0]='\0';
        }
        else if (buff[0]) {

            /* satellite name in three line format */
            strcpy(data.name,buff);

            /* omit words in parentheses */
            if ((p=strchr(data.name,'('))) *p='\0';
            chop(data.name);
            line=0;
        }
    }
    fclose(fp);

    /* sort tle data by satellite name */
    if (tle->n>0) qsort(tle->data,tle->n,sizeof(tled_t),cmp_tle_data);
    return 1;
}

int tle_pos(gtime_t time, const char *name, const char *satno,
                   const char *desig, const tle_t *tle, const erp_t *erp,
                   double *rs)
{
    gtime_t tutc;
    double tsince,rs_tle[6],rs_pef[6],gmst;
    double R1[9]={0},R2[9]={0},R3[9]={0},W[9],erpv[5]={0};
    int i=0,j,k,stat=1;

    /* binary search by satellite name */
    if (*name) {
        for (i=j=0,k=tle->n-1;j<=k;) {
            i=(j+k)/2;
            if (!(stat=strcmp(name,tle->data[i].name))) break;
            if (stat<0) k=i-1; else j=i+1;
        }
    }
    /* serial search by catalog no or international designator */
    if (stat&&(*satno||*desig)) {
        for (i=0;i<tle->n;i++) {
            if (!strcmp(tle->data[i].satno,satno)||
                !strcmp(tle->data[i].desig,desig)) break;
        }
        if (i<tle->n) stat=0;
    }
    if (stat) {
        trace(3,"no tle data: name=%s satno=%s desig=%s\n",name,satno,desig);
        return 0;
    }
    tutc=gpst2utc(time);

    /* time since epoch (min) */
    tsince=timediff(tutc,tle->data[i].epoch)/60.0;

    /* SGP4 model propagator by STR#3 */
    SGP4_STR3(tsince,tle->data+i,rs_tle);

    /* erp values */
    if (erp) geterp(erp,time,erpv);

    /* GMST (rad) */
    gmst=utc2gmst(tutc,erpv[2]);

    /* TEME (true equator, mean eqinox) -> ECEF (ref [2] IID, Appendix C) */
    R1[0]=1.0; R1[4]=R1[8]=cos(-erpv[1]); R1[7]=sin(-erpv[1]); R1[5]=-R1[7];
    R2[4]=1.0; R2[0]=R2[8]=cos(-erpv[0]); R2[2]=sin(-erpv[0]); R2[6]=-R2[2];
    R3[8]=1.0; R3[0]=R3[4]=cos(gmst); R3[3]=sin(gmst); R3[1]=-R3[3];
    matmul("NN",3,1,3,1.0,R3,rs_tle  ,0.0,rs_pef  );
    matmul("NN",3,1,3,1.0,R3,rs_tle+3,0.0,rs_pef+3);
    rs_pef[3]+=OMGE*rs_pef[1];
    rs_pef[4]-=OMGE*rs_pef[0];
    matmul("NN",3,3,3,1.0,R1,R2,0.0,W);
    matmul("NN",3,1,3,1.0,W,rs_pef  ,0.0,rs  );
    matmul("NN",3,1,3,1.0,W,rs_pef+3,0.0,rs+3);
    return 1;
}
