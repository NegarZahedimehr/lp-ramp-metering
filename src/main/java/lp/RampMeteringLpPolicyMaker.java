package lp;

import beats_link.RampMeteringLimitSet;
import beats_link.RampMeteringPolicyMaker;
import beats_link.RampMeteringPolicySet;
import jaxb.*;
import lp.solver.SolverType;
import network.fwy.FwyNetwork;

/**
 * Created by gomes on 6/5/14.
 */
public class RampMeteringLpPolicyMaker implements RampMeteringPolicyMaker {

    protected FwyNetwork fwy;
    protected ProblemRampMetering LP;

    public RampMeteringLpPolicyMaker(network.beats.Network net, FundamentalDiagramSet fd_set, SplitRatioSet split_ratios, ActuatorSet actuator_set,int K_dem,int K_cool,double eta,double sim_dt_in_seconds) throws Exception{
        fwy = new FwyNetwork(net,fd_set,actuator_set);
        fwy.set_split_ratios(split_ratios);
        LP = new ProblemRampMetering(fwy,K_dem,K_cool,eta,sim_dt_in_seconds);
    }

    public void set_data(InitialDensitySet ic,DemandSet demand_set) throws Exception{
        fwy.set_ic(ic);
        fwy.set_demands(demand_set);

        LP.set_rhs_from_fwy(fwy);
    }

    public void set_density_link_network(Long link_id,double density_value){
        fwy.set_density_without_jaxb(link_id, density_value);
//        LP.set_rhs_for_link(fwy, link_id);
    }

    public void set_demand_link_network(Long link_id,Double demand,double demand_dt){
        fwy.set_demand_without_jaxb(link_id, demand,LP.sim_dt_in_seconds);
//        LP.set_rhs_for_link(fwy, link_id);
    }

    public void set_rhs_link(Long link_id,Double demand){
        LP.set_rhs_for_link(fwy,link_id ,demand);
    }

    public RampMeteringSolution solve(SolverType solver_type) throws Exception {
        return new RampMeteringSolution(LP,fwy,solver_type);
    }

    public void printLP(){
        System.out.println(LP);
    }







    @Override
    public RampMeteringPolicySet givePolicy(Network net, FundamentalDiagramSet fd, DemandSet demand, SplitRatioSet splitRatios, InitialDensitySet ics, RampMeteringLimitSet control, Double dt) {


        return null;
    }

    public FwyNetwork getFwy() {
        return fwy;
    }


}
