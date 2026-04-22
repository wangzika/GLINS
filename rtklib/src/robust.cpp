//
// Created by wangchuji on 2022/8/13.
//


#include "rtklib.h"
#define MAX(x,y)    ((x)>=(y)?(x):(y))
static FILE* lossfp = nullptr;
extern void initlossfp(char* path)
{
    char errpath[1024] = { 0 };
    strcpy(errpath, path);
    strcat(errpath, ".loss");
    lossfp = fopen(errpath, "w");
}
extern void closelossfp()
{
    fclose(lossfp);
}
extern double Wi(int fname, double v, double k0, double k1)
{
    double a;
    switch (fname)
    {
    case RMODE_IGG1://IGG1函数
        v = fabs(v);
        if (v <= k0)return 1.0;
        if (v > k1)return 0.00000000001;
        return k0 / v;
    case RMODE_IGG3://IGG3函数
        v = fabs(v);
        if (v <= k0)return 1.0;
        if (v > k1)return 0.00000000001;
        a = (k1 - v) / (k1 - k0);
        return k0 / v * a * a;
    case RMODE_HUBER: // Huber函数
        v = fabs(v);
        if (v <= k0)return 1.0;
        return k0 / v;
    case RMODE_CAUCHY: //Cauchy
        v = fabs(v);
        return 1 / (1 + (v * v) / (k0 * k0));
    default:
        return 1.0;
    }
}
/***
 * 用于最小二乘的抗差
 * P_robust = P * wi
 * Qxx = H * P * H
 * Qvv = R - H * Qxx * H
 * @param H0
 * @param v0
 * @param P
 * @param n 用于多普勒的话，等于4
 * @param m 残差个数
 * @param p
 * @param H
 * @param v
 * @param opt
 */
extern void robust_lsq(const double* H0, const double* v0, const double* P, int n, int m, int p, double* H, double* v, const prcopt_t* opt, int iter)
{
    double* Qvv, * QH0, Q[16], sigma0;
    double wi, wj;
    int i, j, k;
    int info;
    Qvv = mat(m, m);
    QH0 = mat(n, m);

    for (i = 0; i < m; ++i)
    {
        v[i] = v0[i] / sqrt(P[i + i * p]);
        for (j = 0; j < 4; j++)
        {
            H[j + i * 4] = H0[j + i * 4] / sqrt(P[i + i * p]);
        }
    }

    //    matprintT("H",H,n,m,15,10);
    if (opt->robustopt != RMODE_NONE && iter > 0)
    {
        sigma0 = sqrt(dot(v, v, m) / (m - 1));
        matmul("NT", n, n, m, 1.0, H, H, 0.0, Q);

        /*
         * Qv = R-H*Q*H
         */
        if (!(info = matinv(Q, 4)))
        {
            //            matprintT("Q",Q,n,n,15,10);
            matmul("NN", n, m, n, 1.0, Q, H0, 0.0, QH0);
            matmul("TN", m, m, n, 1.0, H0, QH0, 0.0, Qvv);
        }


        //        matprintT("Qvv",Qvv,m,m,15,10);

        //        matprintT("P",P,p,p,15,10);
        for (i = 0; i < m; i++)
        {
            Qvv[i + i * m] = P[i + i * p] - Qvv[i + i * m];
        }
        //        sleepms(1000);
        //        matprintT("Qvv",Qvv,m,m,15,10);
        //        sleepms(1000);
        for (i = 0; i < m; i++)
        {
            // calculate wi
            Qvv[i + i * m] = Qvv[i + i * m] < 0.00000001 ? 0.00000001 : Qvv[i + i * m];
            wi = Wi(opt->robustopt, v0[i] / (sigma0 * sqrt(Qvv[i + i * m])), opt->k0, opt->k1);
            //            printf("wi: %lf v: %lf\n", wi,v0[i] / (sigma0 * sqrt(Qvv[i + i * m])));
            v[i] = v[i] * sqrt(wi);
            for (j = 0; j < 4; j++)
            {
                H[j + i * 4] = H[j + i * 4] * sqrt(wi);
            }
        }
    }
    //    printf("\n");
    //    sleepms(1000);
    //    matprint(v,1,m,15,10);
    free(Qvv); free(QH0);
}

extern double robust(double* x, double* H, double* v, double* P, double* R, int n, int m, double* RR, const prcopt_t* opt, int* type)
{
    double* Qv, * P_p, * R_inv, * H_, * P_, * F;
    int i, j, k, info, * ix;
    double wi, wj;
    double v0_max = 0;
    double v0_min = 0;
    Qv = mat(m, m);
    P_p = mat(m, m); R_inv = mat(m, m);
    ix = imat(n, 1); for (i = k = 0; i < n; i++) if (P[i + i * n] > 0.0) ix[k++] = i;
    H_ = mat(k, m);F = mat(k, m);P_ = mat(k, k);
    for (i = 0; i < k; i++)
    {
        for (j = 0;j < k;j++) P_[i + j * k] = P[ix[i] + ix[j] * n];
        for (j = 0; j < m; j++) H_[i + j * k] = H[ix[i] + j * n];
    }

    //    matprint(H_,k,m,10,4);
    //    printf("\n");
    //    printf("before robust: R=\n");
    //    matprint(R,m,m,10,4);
    matcpy(R_inv, R, m, m);
    if ((info = matinv(R_inv, m)))
    {
        printf("ERROR: matinv(R)\n");
        system("pause");
    }
    //    matprint(R_inv,m,m,10,4);
    //    printf("\n");
        /*
         * Qv = R-H*Pp*H
         */
    matcpy(Qv, R, m, m);
    matmul("NN", k, m, k, 1.0, P_, H_, 0.0, F);
    matmul("TN", m, m, k, 1.0, H_, F, 1.0, Qv);

    //    printf("Qv=\n");
    //    matdiaprint(Qv,m,10,12);
    double vv = dot(v, v, m);
    double qvv = 0;
    for (i = 0; i < m; i++)
    {
        double v0 = fabs(v[i] / (sqrt(Qv[i + i * m])));
        if (v0_max == 0.0 || v0_max < v0) v0_max = v0;
        if (v0_min == 0.0 || v0_min > v0) v0_min = v0;
        qvv += Qv[i + i * m];
    }

    //    printf("v0 %.5lf\n",sqrt(vv/qvv));


    for (i = 0; i < m; i++)
    {
        // calculate wi
        Qv[i + i * m] = Qv[i + i * m] < 0.00000001 ? 0.00000001 : Qv[i + i * m];
        if (type[i] == 0)
        {// carrier phase
            wi = Wi(opt->robustopt, v[i] / (sqrt(Qv[i + i * m])), MAX(v0_min, opt->k0), opt->k1);//opt->k0,opt->k1);
        }
        else        
{
            wi = Wi(opt->robustopt, v[i] / (sqrt(Qv[i + i * m])), MAX(v0_min, opt->k0), opt->k1);//opt->k0,opt->k1);
        }
        //        fprintf(lossfp,"%.3f %10.3f\n",abs(v[i] / (sqrt(Qv[i + i*m]))),wi);
        //        printf("v=%lf\n",fabs(v[i] / (sqrt(Qv[i + i*m]))));
        for (j = 0; j < m; j++)
        {
            // calculate wj
            Qv[j + j * m] = Qv[j + j * m] < 0.00000001 ? 0.00000001 : Qv[j + j * m];
            if (type[j] == 0)
            {// carrier phase
                wj = Wi(opt->robustopt, abs(v[j] / (sqrt(Qv[j + j * m]))), MAX(v0_min, opt->k0), opt->k1);//opt->k0,opt->k1);
            }
            else
            {
                wj = Wi(opt->robustopt, abs(v[j] / (sqrt(Qv[j + j * m]))), MAX(v0_min, opt->k0), opt->k1);//opt->k0,opt->k1);
            }
            //			wj = Wi(opt->robustopt,abs(v[j] / (sqrt(Qv[j + j*m]))),opt->k0,opt->k1);
                        //printf("wi=%lf  wj=%lf\n",wi,wj);
            P_p[j + i * m] = R_inv[j + i * m] * sqrt(wi * wj);
        }
    }
    matcpy(RR, P_p, m, m);
    //    matprint(RR,m,m,10,4);
    //    printf("after robust: R=\n");
    //    matprint(RR,m,m,10,4);
    if ((info = matinv(RR, m)))
    {
        printf("ERROR: matinv(P_p)\n");
        system("pause");
    }
    //    fflush(lossfp);
    free(Qv);free(P_p); free(R_inv);
    free(ix);free(H_);free(P_);free(F);
    return Wi(opt->robustopt, sqrt(vv / qvv), opt->k0, opt->k1);
}