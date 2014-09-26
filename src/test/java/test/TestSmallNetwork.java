package test;

import jaxb.*;
import lp.RampMeteringLpPolicyMaker;
import lp.RampMeteringSolution;
import lp.solver.SolverType;
import network.beats.Network;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertNotNull;

public class TestSmallNetwork {

    private Scenario scenario;
    private double sim_dt_in_seconds = 3d;
    private double K_dem_seconds = 9d;
    private double K_cool_seconds = 15d;
    private double eta = .1d;
    private int num_links;
    private int num_segments;
//    private double[] initialDensityVector;
    private ArrayList<Long> link_ids;

    private SolverType solver_type = SolverType.LPSOLVE;

    @Before
    public void setUp() throws Exception {
        String config = "data/config/smallNetwork.xml";
        scenario = factory.ObjectFactory.getScenario(config);
    }

    @Test
    public void testSmallNetwork() throws Exception {

        int K_dem = (int) Math.round(K_dem_seconds / sim_dt_in_seconds);
        int K_cool = (int) Math.round(K_cool_seconds / sim_dt_in_seconds);

        Network net = (Network) scenario.getNetworkSet().getNetwork().get(0);
        FundamentalDiagramSet fds = scenario.getFundamentalDiagramSet();
        ActuatorSet actuators = scenario.getActuatorSet();
        SplitRatioSet split_ratios = scenario.getSplitRatioSet();
        RampMeteringLpPolicyMaker policy_maker = new RampMeteringLpPolicyMaker(net, fds, split_ratios, actuators, K_dem, K_cool, eta, sim_dt_in_seconds);

        ArrayList<String> errors = policy_maker.getFwy().check_CFL_condition(sim_dt_in_seconds);
        if (!errors.isEmpty()) {
            System.err.print(errors);
            throw new Exception("CFL error");
        }

        link_ids = policy_maker.getFwy().get_link_ids();
        num_links = link_ids.size();
        num_segments = policy_maker.getFwy().num_segments;

        for(int i=0;i<num_links;i++){
            Double densityValue = 0.15;
            Double demandValue = 1.5;
            Double jamDensity = policy_maker.getFwy().get_link_jam_density(link_ids.get(i),fds,net);
            if (densityValue > jamDensity) {
                throw new Exception("Initial density is greater than jam density");
            }

            policy_maker.set_density_link_network(link_ids.get(i),densityValue);
            policy_maker.set_demand_link_network(link_ids.get(i),demandValue,sim_dt_in_seconds);
            policy_maker.set_rhs_link(link_ids.get(i),demandValue);

        }

//        InitialDensitySet ics = scenario.getInitialDensitySet();

//        DemandSet demands = scenario.getDemandSet();
//        policy_maker.set_data(ics, demands);
        RampMeteringSolution sol = policy_maker.solve(solver_type);

        //policy_maker.printLP();

        sol.print_to_file("SmallNetwork", RampMeteringSolution.OutputFormat.matlab);
        sol.print_to_file("SmallNetwork", RampMeteringSolution.OutputFormat.text);

//        System.out.println(sol);

        assertNotNull(sol);
        assertEquals(true, sol.is_CTM());

    }
}


