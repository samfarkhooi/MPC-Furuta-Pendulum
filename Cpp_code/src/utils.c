#include "daqp.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>

int update_ldp(const int mask, DAQPWorkspace *work){
    // TODO: copy dimensions from work->qp? 
    int error_flag, i;
    int do_activate = 0;

    /** Update constraint sense **/
    if(mask&UPDATE_sense){
        if(work->qp->sense == NULL) // Assume all constraints are "normal" inequality constraints
            for(i=0;i<N_CONSTR;i++) work->sense[i] = 0;
        else{
            for(i=0;i<N_CONSTR;i++) work->sense[i] = work->qp->sense[i];
            do_activate = 1;
        }
    }

    /** Update Rinv **/
    if(mask&UPDATE_Rinv){
        error_flag = update_Rinv(work);
        if(error_flag<0)
            return error_flag;
    }
    /** Update M **/
    if(mask&UPDATE_Rinv||mask&UPDATE_M){
        update_M(work,mask);
        normalize_M(work);
    }

    /** Update v **/
    if(mask&UPDATE_Rinv||mask&UPDATE_v){
        update_v(work->qp->f,work,mask);
    }

    // Normalize Rinv
    if(mask&UPDATE_Rinv){
        normalize_Rinv(work);
    }

    /** Update d **/
    if(mask&UPDATE_Rinv||mask&UPDATE_M||mask&UPDATE_v||mask&UPDATE_d){
#ifndef DAQP_ASSUME_VALID
        c_float diff;
        for(i =0;i<N_CONSTR;i++){
            if(IS_IMMUTABLE(i)) continue;
            diff = work->qp->bupper[i] - work->qp->blower[i];
            // Check for trivial infeasibility
            if ( diff < -work->settings->primal_tol ){
                return EXIT_INFEASIBLE;
            }
            // Check for unmarked equality constraint (blower == bupper)
            else if ( diff < work->settings->zero_tol ){
                work->sense[i] |= ACTIVE + IMMUTABLE;
                do_activate = 1;
            }
        }
#endif
        update_d(work);
    }

#ifdef SOFT_WEIGHTS
    // TODO: Use mask or something to avoid scaling something more times... 
    if(work->d_ls != NULL && work->scaling !=NULL){
        for(i=0;i<N_CONSTR; i++){
            work->d_ls[i]*=work->scaling[i];
            work->d_us[i]*=work->scaling[i];
            work->rho_ls[i]/=SQUARE(work->scaling[i]);
            work->rho_us[i]/=SQUARE(work->scaling[i]);
        }
    }
#endif

    // Make sure activate constraints are activated
    if(do_activate){
        reset_daqp_workspace(work);
        error_flag = activate_constraints(work);
        if(error_flag<0)
            return error_flag;
    }

    return 0;
}

int update_Rinv(DAQPWorkspace *work){
    int i,j,k,disp,disp2,disp3;
    const int n = NX; 
        // Check if diagonal
    int is_diagonal = 1;
    for (i=0,disp=1; i<n; i++, disp+=i+1){
        for (j=1; j<n-i; j++,disp++) {
            if(work->qp->H[disp] > 1e-12 || work->qp->H[disp] < -1e-12){
                is_diagonal=0;
                break;
            }
        }
        if(is_diagonal == 0) break;
    }

    // If diagonal, just keep track of variable scaling and use Rinv = I
    if(is_diagonal==1){
        if(work->Rinv != NULL){
            work->RinvD = work->Rinv;
            work->Rinv = NULL;
        }
        c_float Hi;
        i=0; disp=0;
        if(work->scaling != NULL){
            for(;i<N_SIMPLE;i++,disp+=n){ // Combine with settings scaling
                Hi = work->qp->H[disp++]+work->settings->eps_prox;
                if (Hi <= 0) return EXIT_NONCONVEX;
                Hi = sqrt(Hi);
                work->RinvD[i] = 1/Hi;
                work->scaling[i] = Hi;
            }
        }
        for(;i<n;i++,disp+=n){
            Hi = work->qp->H[disp++] + work->settings->eps_prox;
            if (Hi <= 0) return EXIT_NONCONVEX;
            Hi = sqrt(Hi);
            work->RinvD[i] = 1/Hi;
        }
        return 1;
    }
    // Make sure Rinv can be assinged if not diagonal
    //(necessary if H change from diagonal to non-diagonal)
    if(work->RinvD != NULL && work->Rinv ==NULL){
        work->Rinv= work->RinvD;
        work->RinvD = NULL;
    }


    // Cholesky
    for (i=0,disp=0,disp3=0; i<n; disp+=n-i,i++,disp3+=i) {
        // Diagonal element
        work->Rinv[disp] = work->qp->H[disp3++]+work->settings->eps_prox;// Add regularization
        for (k=0,disp2=i; k<i; k++,disp2+=n-k) 
            work->Rinv[disp] -= work->Rinv[disp2]*work->Rinv[disp2];
        if (work->Rinv[disp] <= 0) return EXIT_NONCONVEX; // Not positive definite 

        work->Rinv[disp] = sqrt(work->Rinv[disp]);

        // Off-diagonal elements
        for (j=1; j<n-i; j++) {
            work->Rinv[disp+j]=work->qp->H[disp3++];
            for (k=0,disp2=i; k<i; k++,disp2+=n-k)
                work->Rinv[disp+j] -= work->Rinv[disp2]*work->Rinv[disp2+j];
            work->Rinv[disp+j] /= work->Rinv[disp];
        }
        // Store 1/r_ii instead of r_ii 
        // to get multiplication instead division when forward/backward substituting
        work->Rinv[disp] = 1/work->Rinv[disp]; 
    }

    // Compute Rinv (store in R) by Rinv = R\I 
    for(k=0,disp=0;k<n;k++){
        disp2=disp;
        work->Rinv[disp]=work->Rinv[disp2++]; // Break out first iteration to get rhs
        for(j=k+1;j<n;j++)
            work->Rinv[disp2++]*=-work->Rinv[disp];
        disp++;
        for(i=k+1;i<n;i++,disp++){
            work->Rinv[disp]*=work->Rinv[disp2++];
            for(j=1;j<n-i;j++)
                work->Rinv[disp+j]-=work->Rinv[disp2++]*work->Rinv[disp];
        }
    }
    return 1;
}

void update_M(DAQPWorkspace *work, const int mask){
    int i,j,k,disp,disp2;
    const int n = NX;
    const int mA = N_CONSTR-N_SIMPLE;
    int stop_id =  (UPDATE_Rinv &mask) ? NX : NX-N_SIMPLE;
    if(work->Rinv != NULL){
        for(k = 0,disp2=n*mA-1;k<mA;k++,disp2-=n){
            disp=ARSUM(n);
            for(j = 0; j< stop_id ; ++j){
                for(i=0;i<j;++i)
                    work->M[disp2-i] += work->Rinv[--disp]*work->qp->A[disp2-j];
                work->M[disp2-j]=work->Rinv[--disp]*work->qp->A[disp2-j];
            }
            for(; j<n; ++j){// Take into account scaling in Rinv 
                for(i=0;i<j;++i)
                    work->M[disp2-i] += (work->Rinv[--disp]/work->scaling[n-j-1])*work->qp->A[disp2-j];
                work->M[disp2-j]=(work->Rinv[--disp]/work->scaling[n-j-1])*work->qp->A[disp2-j];
            }
        }
    }
    else{
        if(work->RinvD == NULL){ // Copy A to M 
            for(k = 0,disp=0;k<mA;k++){
                for(i=0;i<NX;i++,disp++)
                    work->M[disp] = work->qp->A[disp];
            }
        }
        else{
            for(k = 0,disp=0;k<mA;k++){
                for(i=0;i<NX;i++,disp++)
                    work->M[disp] = work->qp->A[disp]*work->RinvD[i];
            }
        }
    }

    reset_daqp_workspace(work); // Internal factorizations need to be redone!
}

void update_v(c_float *f, DAQPWorkspace *work, const int mask){
    int i,j,disp;
    const int n = NX;
    if(work->v == NULL || f == NULL) return;
    if(work->Rinv == NULL){// Rinv = I => v = R'\v = f
        if(work->RinvD != NULL)
            for(i=0;i<n;++i) work->v[i] = f[i]*work->RinvD[i];
        else
            for(i=0;i<n;++i) work->v[i] = f[i];
        return;
    }
    int stop_id =  (mask & UPDATE_Rinv) ? 0 : N_SIMPLE;
    for(j=n-1,disp=ARSUM(n);j>=stop_id;j--){
        for(i=n-1;i>j;i--)
            work->v[i] +=work->Rinv[--disp]*f[j];
        work->v[j]=work->Rinv[--disp]*f[j];
    }
    for(;j>=0;j--){// Take into accoutn scaling in Rinv
        for(i=n-1;i>j;i--)
            work->v[i] +=(work->Rinv[--disp]/work->scaling[j])*f[j];
        work->v[j]=(work->Rinv[--disp]/work->scaling[j])*f[j];
    }
}

void update_d(DAQPWorkspace *work){
    /* Compute d  = b+M*v */
    int i,j,disp;
    c_float sum;
    const int n = NX;
    work->reuse_ind = 0; // RHS of KKT system changed => cannot reuse intermediate results
    // Take into scaling of constraints
    if(work->scaling != NULL){
        for(i = 0;i<N_CONSTR;i++){
            work->dupper[i] = work->qp->bupper[i]*work->scaling[i];
            work->dlower[i] = work->qp->blower[i]*work->scaling[i];
        }
    }
    else{
        for(i = 0;i<N_CONSTR;i++){
            work->dupper[i] = work->qp->bupper[i];
            work->dlower[i] = work->qp->blower[i];
        }
    }

    if(work->v == NULL) return;
    // Simple bounds 
    if(work->Rinv !=NULL){
        for(i = 0,disp=0;i<N_SIMPLE;i++){
            for(j=i, sum=0;j<n;j++)
                sum+=work->Rinv[disp++]*work->v[j];
            work->dupper[i]+=sum;
            work->dlower[i]+=sum;
        }
    }else{
        for(i = 0,disp=0;i<N_SIMPLE;i++){
            work->dupper[i]+=work->v[i];
            work->dlower[i]+=work->v[i];
        }
    }
    //General bounds
    for(i = N_SIMPLE, disp=0;i<N_CONSTR;i++){
        for(j=0, sum=0;j<n;j++)
            sum+=work->M[disp++]*work->v[j];
        work->dupper[i]+=sum;
        work->dlower[i]+=sum;
    }

}

void normalize_Rinv(DAQPWorkspace* work){
    int i,j,disp;
    c_float scaling_i;
    // Normalize simple constraints
    if(work->Rinv !=NULL){
        for(i=0, disp=0; i < N_SIMPLE;i++){
            scaling_i = 0;
            for(j=i; j < NX; j++,disp++){
                scaling_i+=work->Rinv[disp]*work->Rinv[disp];
            }
            scaling_i = 1/sqrt(scaling_i);
            work->scaling[i] = scaling_i; // Need to save to correctly retrieve solution
            for(j=i,disp-=(NX-i); j < NX; j++,disp++)
                work->Rinv[disp]*= scaling_i;
        }
    }
}
void normalize_M(DAQPWorkspace* work){
    int i,j,disp;
    c_float scaling_i;
    // Normalize general constraints 
    for(i=N_SIMPLE, disp=0;i<N_CONSTR;i++){
        scaling_i = 0;
        for(j=0;j<NX;disp++,j++)
            scaling_i+=work->M[disp]*work->M[disp];
        if(scaling_i < work->settings->zero_tol){
            work->sense[i] = IMMUTABLE; // ignore zero-row constraint
            continue; // TODO: mark infeasibility if dupper & dlower are nonzero
        }
        scaling_i = 1/sqrt(scaling_i);
        work->scaling[i]=scaling_i;
        for(j=0, disp-=NX;j<NX;j++,disp++)
            work->M[disp]*=scaling_i;
    }
}

/* Remove Minrep */
void daqp_minrep_work(int* is_redundant, DAQPWorkspace* work){
    int i,j,exitflag;

    for(i=0; i < work->m; i++)
        is_redundant[i] = -1;

    for(i=0; i < work->m; i++){
        if(is_redundant[i] != -1 || (work->sense[i]&IMMUTABLE) !=  0) continue;
        reset_daqp_workspace(work);
        work->sense[i] = 5;
        add_constraint(work,i,1.0);
        //work->dupper[i] += tol_weak; TODO support weaky infeasible constraints
        exitflag = daqp_ldp(work);
        if(exitflag== EXIT_INFEASIBLE){
            is_redundant[i] = 1;
            work->sense[i] &=~ACTIVE; // deactive (remains immutable, so will be ignored)
        }
        else{
            is_redundant[i] = 0;
            work->sense[i] &=~IMMUTABLE;
            if(exitflag==EXIT_OPTIMAL)
                for(j=0; j < work->n_active; j++) // all active constraint must also be nonredundant
                    is_redundant[work->WS[j]] = 0;
        }
        // work->dupper[i] -= tol_weak; // TODO support weakly infeasible constraints
        deactivate_constraints(work);
    }
}

/* Profiling */
#ifdef PROFILING
#ifdef _WIN32
void tic(DAQPtimer *timer){
    QueryPerformanceCounter(&(timer->start));
}
void toc(DAQPtimer *timer){
    QueryPerformanceCounter(&(timer->stop));
}
double get_time(DAQPtimer *timer){
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return (double)(timer->stop.QuadPart - timer->start.QuadPart)/f.QuadPart;
}
#else // not _WIN32 (assume that time.h works) 

void tic(DAQPtimer *timer){
    clock_gettime(CLOCK_MONOTONIC, &(timer->start));
}
void toc(DAQPtimer *timer){
    clock_gettime(CLOCK_MONOTONIC, &(timer->stop));
}

double get_time(DAQPtimer *timer){
    struct timespec diff;
    if ((timer->stop.tv_nsec - timer->start.tv_nsec) < 0) {
        diff.tv_sec  = timer->stop.tv_sec - timer->start.tv_sec - 1;
        diff.tv_nsec = 1e9 + timer->stop.tv_nsec - timer->start.tv_nsec;
    } else {
        diff.tv_sec  = timer->stop.tv_sec - timer->start.tv_sec;
        diff.tv_nsec = timer->stop.tv_nsec - timer->start.tv_nsec;
    }
    return (double)diff.tv_sec + (double )diff.tv_nsec / 1e9;
}
#endif // _WIN32
#endif // PROFILING 
