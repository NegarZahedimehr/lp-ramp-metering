package lp;

import network.beats.Network;
import network.fwy.FwyNetwork;
import network.fwy.FwySegment;
import jaxb.*;
import lp.problem.*;

public class LP_ramp_metering {

    protected FwyNetwork fwy;
    protected double sim_dt_in_seconds;   // time step in seconds
    protected int K;                      // number of time steps (demand+cooldown)
    protected int Kcool;                  // number of cooldown time steps
    protected int I;                      // number of segments
    protected double eta = 1.0;           // objective = TVH - eta*TVM
    protected double gamma = 1d;          // merge coefficient
    protected Linear objective = new Linear();
    protected boolean is_valid;
    protected String validation_message;

    ///////////////////////////////////////////////////////////////////
    // construction
    ///////////////////////////////////////////////////////////////////

    public LP_ramp_metering(Scenario scenario, int K_dem,int K_cool, double sim_dt_in_seconds) throws Exception{

        int i,k;

        // constant information
        Network network = (Network) scenario.getNetworkSet().getNetwork().get(0);
        ActuatorSet actuators = scenario.getActuatorSet();
        FundamentalDiagramSet fds = scenario.getFundamentalDiagramSet();

        // Make the freeway structure
        fwy = new FwyNetwork(network,fds,actuators,sim_dt_in_seconds);

        this.K = K_dem+K_cool;
        this.Kcool = K_cool;
        this.sim_dt_in_seconds = sim_dt_in_seconds;
        this.I = fwy.getSegments().size();

        /* objective function:
           sum_[I][K] n[i][k] + sum_[Im][K] l[i][k] - eta sum_[I][K] f[i][k] - eta sum[Im][K] r[i][k]
         */
        for(i=0;i<I;i++){
            FwySegment seg = fwy.getSegments().get(i);
            for(k=0;k<K;k++){
                objective.add_coefficient(1.0, getVar("n", i, k + 1));
                objective.add_coefficient(-eta, getVar("f", i, k));
            }
            if(seg.is_metered)
                for(k=0;k<K;k++){
                    objective.add_coefficient(1.0, getVar("l", i, k + 1));
                    objective.add_coefficient(-eta, getVar("r", i, k));
                }
        }

        /* onramp conservation
           for each metered i in 0...I-1, k in 0...K-1
            {always}    {k>0}  {always}
           l[i][k+1]  -l[i][k] +r[i][k]
         */
        for(i=0;i<I;i++){
            FwySegment seg = fwy.getSegments().get(i);
            if(seg.is_metered){
                for(k=0;k<K;k++){
                    Linear C = new Linear();
                    C.add_coefficient(+1d, getVar("l", i, k + 1));
                    if(k>0)
                        C.add_coefficient(-1d, getVar("l", i, k));
                    C.add_coefficient(+1d, getVar("r", i, k));
                    seg.ORcons.add(C);
                }
            }
        }

        /* mainline flows - congestion
           for each i in 0...I-2, k in 0...K-1
           {always}       {k>0}                {metered}
           f[i][k] + w[i+1]*n[i+1][k] + w[i+1]*gamma*r[i+1][k]
         */
        for(i=0;i<I-1;i++){
            FwySegment seg = fwy.getSegments().get(i);
            FwySegment next_seg = fwy.getSegments().get(i+1);
            for(k=0;k<K;k++){
                Linear C = new Linear();
                C.add_coefficient(+1d, getVar("f", i, k));
                if(k>0)
                    C.add_coefficient(next_seg.w, getVar("n", i + 1, k));
                if(next_seg.is_metered)
                    C.add_coefficient(next_seg.w * gamma, getVar("r", i + 1, k));
                seg.MLcng.add(C);
            }
        }

        /* onramp flow demand
           for each metered i in 0...I-1, k in 0...K-1
           {always}   {k>0}
           r[i][k] - l[i][k]
         */
        for(i=0;i<I;i++){
            FwySegment seg = fwy.getSegments().get(i);
            if(seg.is_metered){
                for(k=0;k<K;k++){
                    Linear C = new Linear();
                    C.add_coefficient(+1d, getVar("r", i, k));
                    if(k>0)
                        C.add_coefficient(-1d, getVar("l", i, k));
                    seg.ORdem.add(C);
                }
            }
        }

        // VALIDATION ............................................
        is_valid = true;
        validation_message = "Error in scenario.";

        if(scenario.getSettings().getUnits().compareToIgnoreCase("si")!=0){
            is_valid = false;
            validation_message = validation_message.concat("\n+ Wrong units.");
        }


    }

    protected LP_solution solve(InitialDensitySet ic, DemandSet demand_set, SplitRatioSet split_ratios) throws Exception {

        /* copy input to network.beats.fwy structure */
        fwy.set_ic(ic);
        fwy.set_demands(demand_set,sim_dt_in_seconds,K,Kcool);
        fwy.set_split_ratios(split_ratios,sim_dt_in_seconds,K,Kcool);

        Problem LP = cast_to_Problem();

        return new LP_solution(LP,fwy,K);
    }

    private Problem cast_to_Problem(){

        int i,k;
        double rhs;

        /* generate problem, assign objective function */
        Problem LP = new Problem();
        LP.setObjective(objective, OptType.MIN);

        /* assign rhs, add constraints for each segment */
        for(i=0;i<I;i++){

            FwySegment seg = fwy.getSegments().get(i);

            /* mainline conservation (LHS)
               for each i in 0...I-1, k in 0...K-1
                {always}    {k>0}     {i>0}   {metered}       {betabar[i][k]>0}
               n[i][k+1] -n[i][k] -f[i-1][k]  -r[i][k] +inv(betabar[i][k])*f[i][k]

              RHS:
                    {k>0}  {k==0}
               LHS =  0  + n[i][0]
            */

            for(k=0;k<K;k++){
                Linear C = new Linear();

                // LHS
                C.add_coefficient(+1d, getVar("n", i, k + 1));
                if(k>0)
                    C.add_coefficient(-1d, getVar("n", i, k));
                if(i>0)
                    C.add_coefficient(-1d, getVar("f", i - 1, k));
                if(seg.is_metered)
                    C.add_coefficient(-1d, getVar("r", i, k));
                if(seg.betabar(k)!=0d)
                    C.add_coefficient(+1 / seg.betabar(k), getVar("f", i, k));

                // relation
                C.set_relation(Relation.EQ);

                // RHS
                rhs = k==0 ? seg.no : 0;
                rhs += !seg.is_metered ? seg.d(k) : 0;
                C.set_rhs(rhs);
                LP.add_constraint(C);
            }

            /* RHS: or conservation
               for each metered i in 0...I-1, k in 0...K-1
                     {always}  {k==0}
               LHS = d[i][k] + l[i][0]
             */
            if(seg.is_metered)
                for(k=0;k<K;k++){
                    rhs = seg.d(k);
                    rhs += k==0 ? seg.lo : 0;
                    LP.add(seg.ORcons.get(k), "=", rhs);
                }


            /* mainline flows - freeflow
               for each i in 0...I-1, k in 0...K-1
               {always}           {k>0}                       {metered}
               f[i][k] -betabar[i][k]*v[i]*n[i][k] - betabar[i][k]*v[i]*gamma*r[i][k]

              RHS: ml flow freeflow
               for each i in 0...I-1, k in 0...K-1
                                {k==0}                      {!metered}
               LHS <= betabar[i][0]*v[i]*n[i][0] + betabar[i][k]*v[i]*gamma*d[i][k]
             */
            for(k=0;k<K;k++){
                Linear C = new Linear();

                // LHS
                C.add_coefficient(+1d, getVar("f", i, k));
                if(k>0 && seg.betabar(k)>0)
                    C.add_coefficient(-seg.betabar(k) * seg.vf, getVar("n", i, k));
                if(seg.is_metered && seg.betabar(k)>0)
                    C.add_coefficient(-seg.betabar(k) * seg.vf * gamma, getVar("r", i, k));

                // RHS
                rhs = k==0 ? seg.betabar(0)*seg.vf*seg.no : 0;
                rhs += !seg.is_metered ? seg.betabar(k)*seg.vf*gamma*seg.d(k) : 0;
                LP.add(C, "<=", rhs);
            }

            /* RHS: ml flow congestion
               for each i in 0...I-2, k in 0...K-1
                         {always}              {k=0}             {!metered}
               LHS <= w[i+1]*njam[i+1] - w[i+1]*n[i+1][0] - w[i+1]*gamma*d[i+1][k]
             */
            if(i<I-1){
                FwySegment next_seg = fwy.getSegments().get(i+1);
                for(k=0;k<K;k++){
                    rhs = next_seg.w*next_seg.n_max;
                    rhs += k==0 ? -next_seg.w*next_seg.no : 0;
                    rhs += !next_seg.is_metered ? -gamma*next_seg.w*next_seg.d(k) : 0;
                    LP.add(seg.MLcng.get(k), "<=", rhs);
                }
            }

            /* RHS: or flow demand
               for each metered i in 0...I-1, k in 0...K-1
                      {always}
               LHS <= d[i][k]
             */
            if(seg.is_metered)
                for(k=0;k<K;k++){
                    rhs = k==0 ? seg.lo : 0;
                    rhs += seg.d(k);
                    LP.add(seg.ORdem.get(k), "<=", rhs);
                }

            /* BOUND: mainline capacity
               for each i in 0...I-1, k in 0...K-1
               f[i][k] <= f_max[i]
             */
            for(k=0;k<K;k++)
                LP.setVarUpperBound(getVar("f",i,k),seg.f_max);

            /* BOUND: maximum metering rate
               for each metered i in 0...I-1, k in 0...K-1
               r[i][k] <= rmax[i]
             */
            if(seg.is_metered)
                for(k=0;k<K;k++)
                    LP.setVarUpperBound(getVar("r", i, k), seg.r_max);

            /* BOUND: onramp flow positivity
               for each metered i in 0...I-1, k in 0...K-1
               r[i][k] >= 0
             */
            if(seg.is_metered)
                for(k=0;k<K;k++)
                    LP.setVarLowerBound(getVar("r",i,k),0);

            /* BOUND: queue length limit
               for each metered i in 0...I-1, k in 0...K-1
               l[i][k] <= lmax[i]
             */
            if(seg.is_metered && !seg.l_max.isInfinite())
                for(k=0;k<K;k++)
                    LP.setVarUpperBound(getVar("l",i,k+1),seg.l_max);
        }

        return LP;
    }

    public static String getVar(String name,int index,int timestep){
        return name+"_"+index+"_"+timestep;
    }
}