package edu.berkeley.path.lprm.fwy;

import edu.berkeley.path.beats.jaxb.*;
import edu.berkeley.path.lprm.graph.LpLink;
import edu.berkeley.path.lprm.graph.LpNetwork;
import edu.berkeley.path.lprm.graph.LpNode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public final class FwyNetwork {

    protected int num_segments;           // number of segments
    protected int num_frs;
    protected double gamma = 1d;          // merge coefficient

    protected ArrayList<FwySegment> segments;
    protected ArrayList<Long> ml_link_id;
    protected ArrayList<Long> fr_link_id;
    protected ArrayList<Long> or_link_id;
    protected ArrayList<Long> or_source_id;
    protected ArrayList<Long> fr_node_id;

    ///////////////////////////////////////////////////////////////////
    // construction
    ///////////////////////////////////////////////////////////////////

    public FwyNetwork(LpNetwork network,FundamentalDiagramSet fds,ActuatorSet actuatorset) throws Exception{

        segments = new ArrayList<FwySegment>();
        ml_link_id = new ArrayList<Long>();
        fr_link_id = new ArrayList<Long>();
        or_link_id = new ArrayList<Long>();
        or_source_id = new ArrayList<Long>();
        fr_node_id = new ArrayList<Long>();

        // find first mainline link, then iterate downstream until you reach the end
        LpLink link = find_first_fwy_link(network);
        while(link!=null){
            LpLink onramp = begin_node_entering_onramp(link);
            LpLink offramp = end_node_offramp(link);
            FundamentalDiagram fd = get_fd_for_link(link,fds);
            Actuator actuator = get_onramp_actuator(onramp,actuatorset);
            LpLink onramp_source = get_onramp_source(onramp);
            segments.add(new FwySegment(link,onramp,offramp,fd,actuator));
            ml_link_id.add(link.getId());
            or_link_id.add(onramp==null?null:onramp.getId());
            or_source_id.add(onramp_source==null?null:onramp_source.getId());
            fr_link_id.add(offramp==null?null:offramp.getId());
            fr_node_id.add(offramp == null ? null : offramp.getBegin().getId());
            link = next_freeway_link(link);
        }

        num_segments = segments.size();
        num_frs = fr_link_id.size();

    }

    ///////////////////////////////////////////////////////////////////
    // validate
    ///////////////////////////////////////////////////////////////////

    public String check_cfl_condition(double dt){
        String errors = "";
        for(int i=0;i<segments.size();i++){
            FwySegment seg = segments.get(i);
            if(seg.get_vf_link_per_sec()*dt>1.0)
                errors += String.format("vf=%f in segment %d\n",seg.get_vf_link_per_sec()*dt,i);
            if(seg.get_w_link_per_sec()*dt>1.0)
                errors += String.format("w=%f in segment %d\n",seg.get_vf_link_per_sec()*dt,i);
        }
        return errors;
    }

    ///////////////////////////////////////////////////////////////////
    // get
    ///////////////////////////////////////////////////////////////////

    public ArrayList<FwySegment> get_segments(){
        return segments;
    }

    public FwySegment get_segment(int i){
        return segments.get(i);
    }

    public int get_num_segments(){
        return num_segments;
    }

    public double gamma(){
        return gamma;
    }

    public ArrayList<Long> get_mainline_ids(){
        return ml_link_id;
    }

    public ArrayList<Long> get_offramp_ids(){
        return fr_link_id;
    }

    public ArrayList<Long> get_metered_onramp_ids(){
        ArrayList<Long> x = new ArrayList<Long>();
        for(FwySegment seg : segments)
            if( seg.is_metered )
                x.add(seg.get_on_ramp_link_id());
        return x;
    }

    public double get_njam_veh_per_link(long link_id) {
        for(FwySegment seg : segments){
            if( seg.get_main_line_link_id()==link_id )
                return seg.n_max;
            if( seg.get_on_ramp_link_id() == link_id ){
                return seg.l_max;
            }
        }
        return Double.NaN;
    }

    ///////////////////////////////////////////////////////////////////
    // simulate_no_control
    ///////////////////////////////////////////////////////////////////

//    public FwyStateTrajectory simulate_no_control(double dt, int Kdem, int Kcool){
//
//        int i,k;
//        double t,betabar,demand;
//        int num_steps = Kdem+Kcool;
//
//        FwyStateTrajectory X = new FwyStateTrajectory(this,dt,num_steps);
//
//        for(k=0;k<num_steps;k++){
//            t = k*dt;
//
//            // update onramps
//            for(i=0;i<num_segments;i++){
//
//                FwySegment seg = segments.get(i);
//                demand = k<Kdem ? seg.get_demand_in_vps(t)*dt : 0d;
//
//                X.r[i][k] = !seg.is_metered ? demand :
//                                minmin(
//                                X.l[i][k] + demand ,
//                                seg.r_max*dt ,
//                                seg.n_max - X.n[i][k] );
//
//                X.l[i][k+1] = !seg.is_metered ? 0d : X.l[i][k] + demand - X.r[i][k];
//            }
//
//            // update mainline
//            for(i=0;i<num_segments;i++){
//
//                FwySegment seg = segments.get(i);
//                FwySegment nextseg = i<num_segments-1 ? segments.get(i+1) : null;
//                betabar = 1-seg.get_split_ratio(t);
//
//                X.f[i][k] = minmin(
//                        seg.vf*dt * betabar * (X.n[i][k] + gamma*X.r[i][k])  ,
//                        seg.f_max*dt ,
//                        i<num_segments-1 ? nextseg.w*dt *(nextseg.n_max-X.n[i+1][k] -gamma*X.r[i+1][k]) : Double.POSITIVE_INFINITY );
//
//                X.n[i][k+1] = X.n[i][k]
//                        + (i==0 ? 0d : X.f[i-1][k])
//                        + X.r[i][k]
//                        - X.f[i][k] / betabar ;
//            }
//
//        }
//        return X;
//    }


    /** Simulate the freeway network. Onramp flows can be directly provided in rmap. In this case there
     * is no guarantee of well-behaved CTM evolution. Runs an uncontrolled CTM if rmap==null.
     *
     * @param dt Simulation time step in seconds
     * @param Kdem Number of time steps with positive demand
     * @param Kcool Number of cool-down time steps
     * @param rmap Map of onramp ids to ramp flows
     * @return
     */
    public FwyStateTrajectory simulate(double dt,int Kdem,int Kcool, HashMap<Long,double[]> rmap){

        int i,k;
        double t,betabar,demand;

        // determine number of steps
        int num_steps_rmap = rmap==null ? Integer.MAX_VALUE : rmap.values().iterator().next().length;
        int num_steps = Math.min( Kdem+Kcool , num_steps_rmap );

        // initialize the state
        FwyStateTrajectory X = new FwyStateTrajectory(this,dt,num_steps);

        for(k=0;k<num_steps;k++){
            t = k*dt;

            // update onramps
            for(i=0;i<num_segments;i++){

                FwySegment seg = segments.get(i);
                demand = k<Kdem ? seg.get_demand_in_vps(t)*dt : 0d;

                if(rmap!=null && seg.or_link_id!=null) {      // given ramp flows
                    double [] r = rmap.get(seg.or_link_id);
                    X.r[i][k] = r[k];
                }
                else                // otherwise apply the model
                    X.r[i][k] = !seg.is_metered ? demand :
                            minmin(
                                    X.l[i][k] + demand ,
                                    seg.r_max*dt ,
                                    seg.n_max - X.n[i][k] );

                X.l[i][k+1] = !seg.is_metered ? 0d : X.l[i][k] + demand - X.r[i][k];
            }

            // update mainline
            for(i=0;i<num_segments;i++){

                FwySegment seg = segments.get(i);
                FwySegment nextseg = i<num_segments-1 ? segments.get(i+1) : null;
                betabar = 1-seg.get_split_ratio(t);

                X.f[i][k] = minmin(
                        seg.vf*dt * betabar * (X.n[i][k] + gamma*X.r[i][k])  ,
                        seg.f_max*dt ,
                        i<num_segments-1 ? nextseg.w*dt *(nextseg.n_max-X.n[i+1][k] -gamma*X.r[i+1][k]) : Double.POSITIVE_INFINITY );

                X.n[i][k+1] = X.n[i][k]
                        + (i==0 ? 0d : X.f[i-1][k])
                        + X.r[i][k]
                        - X.f[i][k] / betabar ;
            }

        }
        return X;
    }

    ///////////////////////////////////////////////////////////////////
    // private
    ///////////////////////////////////////////////////////////////////

    private LpLink find_first_fwy_link(LpNetwork network) throws Exception{

        if(!all_are_fwy_or_fr_src_snk(network))
            throw new Exception("Not all links are freeway, onramp, offramps, sources, or sinks");

        // gather links that are freeway sources, or sources that end in simple nodes
        List<LpLink> first_fwy_list = new ArrayList<LpLink>();
        for(LpLink link : network.getLinks()){
            boolean isFirstMainLine = false;
            LpNode end_node = link.getEnd();
            LpNode begin_node = link.getBegin();
            int nInputLinks = begin_node.getnIn();
            if (nInputLinks == 0 && link.isFreeway())
                isFirstMainLine = true;
            if (nInputLinks == 1 && begin_node.getInput(0).isOnramp() && link.isFreeway())
                isFirstMainLine = true;
            //boolean end_node_is_simple = end_node.getnIn()==1 && end_node.getnOut()==1;
            boolean supplies_onramp = end_node.getnOut()>0 ? end_node.getOutput(0).isOnramp() : false;
              if( isFirstMainLine && !supplies_onramp){
                first_fwy_list.add(link);
            LpNode first_freewayLink_begin_node = link.getBegin();
                  LpNode first_freewayLink_end_node = link.getEnd();
              }
        }
        // there must be exactly one of these
        if(first_fwy_list.isEmpty())
            throw new Exception("NO FIRST FWY LINK");

        if(first_fwy_list.size()>1)
            throw new Exception("MULTIPLE FIRST FWY LINKS");

        LpLink first_fwy = first_fwy_list.get(0);

        // if it is a source link, use next
        //if(isSource(first_fwy))
        //    first_fwy = first_fwy.getEnd_node().getOutput_link()[0];

        return first_fwy;
    }

    private boolean all_are_fwy_or_fr_src_snk(LpNetwork network){
        for(LpLink link : network.getLinks()){
            if(!link.isFreeway() && !link.isOnramp() && !link.isOfframp() && !link.isSource() && !link.isSink())
                return false;
        }
        return true;
    }

    private LpLink end_node_offramp(LpLink link){
        for(LpLink olink : link.getEnd().getOutputs()){
            if(olink.isOfframp())
                return olink;
        }
        return null;
    }

    private LpLink next_freeway_link(LpLink link){
        for(LpLink olink : link.getEnd().getOutputs()){
            if(olink.isFreeway())
                return olink;
        }
        return null;
    }

    private LpLink begin_node_entering_onramp(LpLink link){
        for(LpLink ilink : link.getBegin().getInputs()){
            if(ilink.isOnramp())
                return ilink;
        }
        return null;
    }

    private FundamentalDiagram get_fd_for_link(LpLink link, FundamentalDiagramSet fds){
        if(fds==null)
            return null;
        for(FundamentalDiagramProfile fdp : fds.getFundamentalDiagramProfile())
            if(link.getId()==fdp.getLinkId())
                if(fdp.getFundamentalDiagram()!=null && !fdp.getFundamentalDiagram().isEmpty())
                    return fdp.getFundamentalDiagram().get(0);
        return null;
    }

    private Actuator get_onramp_actuator(LpLink onramp, ActuatorSet actuatorset){
        if(actuatorset==null)
            return null;
        if(onramp==null)
            return null;
        for(Actuator actuator : actuatorset.getActuator()){
            if( actuator.getActuatorType().getName().compareTo("ramp_meter")==0 &&
                    actuator.getScenarioElement().getType().compareTo("link")==0 &&
                    actuator.getScenarioElement().getId()==onramp.getId() )
                return actuator;
        }
        return null;
    }

    private LpLink get_onramp_source(LpLink link){
        LpLink rlink = link;
        while(rlink!=null && !rlink.isSource()){
            LpNode node = rlink.getBegin();
            rlink = node.getnIn()==1 ? node.getInput(0) : null;
        }
        return rlink;
    }

    private static double minmin(double a,double b,double c){
        return Math.min(Math.min(a,b),c);
    }

    ///////////////////////////////////////////////////////////////////
    // set
    ///////////////////////////////////////////////////////////////////

    public void set_ic(InitialDensitySet ic) throws Exception {

        // reset everything to zero
        for(FwySegment seg : segments)
            seg.reset_state();

        if(ic==null)
            return;

        // distribute initial condition to segments
        for(Density D : ic.getDensity()){
            int index = ml_link_id.indexOf(D.getLinkId());
            if(index>=0)
                segments.get(index).set_no_in_vpm(Double.parseDouble(D.getContent()));
            index = or_link_id.indexOf(D.getLinkId());
            if(index>=0)
                segments.get(index).set_lo_in_vpm(Double.parseDouble(D.getContent()));
        }
    }

    public void set_density_in_vpm(Long link_id, double density_value)  throws Exception {
        int index = ml_link_id.indexOf(link_id);
        if (index>=0)
            segments.get(index).set_no_in_vpm(density_value);
        index = or_link_id.indexOf(link_id);
        if(index>=0)
            segments.get(index).set_lo_in_vpm(density_value);
    }

    public void set_density_in_veh(Long link_id, double density_value)  throws Exception {
        int index = ml_link_id.indexOf(link_id);
        if (index>=0)
            segments.get(index).set_no_in_veh(density_value);
        index = or_link_id.indexOf(link_id);
        if(index>=0)
            segments.get(index).set_lo_in_veh(density_value);
    }

    public void set_demand_in_vps(Long link_id, Double demand_value, double demand_dt)  throws Exception {
//        FwySegment seg;
//        if(index>=0){
//            seg = segments.get(index);
//            if (seg.has_on_ramp)
//            seg.set_constant_demand_segment(demand_value,index,dt);}
        int index = ml_link_id.indexOf(link_id);
        if (index>=0) {
            FwySegment seg = segments.get(index);
            ArrayList<Double> demand_profile = new ArrayList<Double>();
            demand_profile.add(demand_value);
            seg.set_demands_in_vps(demand_profile,demand_dt);
        }
    }

    public void set_demands(DemandSet demand_set) throws Exception {

        // reset everything to zero
        for(FwySegment seg : segments)
            seg.reset_demands();

        for(DemandProfile dp : demand_set.getDemandProfile()){
            int index = or_source_id.indexOf(dp.getLinkIdOrg());
            double knob = dp.getKnob();
            if(index>=0){
                FwySegment seg = segments.get(index);
                ArrayList<Double> demand = new ArrayList<Double>();
                if(dp.getDemand()!=null){
                    for(Demand d:dp.getDemand()){
                        List<String> strlist = Arrays.asList(d.getContent().split(","));
                        if(demand.isEmpty())
                            for(int i=0;i<strlist.size();i++)
                                demand.add(0d);
                        for(int i=0;i<strlist.size();i++){
                            double val = demand.get(i);
                            val += knob*Double.parseDouble(strlist.get(i))*seg.get_or_lanes();
                            demand.set(i,val);
                        }
                    }
                }
                seg.set_demands_in_vps(demand, dp.getDt());
            }
        }
    }

    public void set_split_ratios(SplitRatioSet srs,boolean enforce_constant_splits) throws Exception{
        int index;

        for(SplitRatioProfile srp : srs.getSplitRatioProfile()){
            index = fr_node_id.indexOf(srp.getNodeId());
            if(index>=0){
                ArrayList<Double> ml_split = new ArrayList<Double>();
                ArrayList<Double> fr_split = new ArrayList<Double>();
                for(Splitratio sr : srp.getSplitratio())
                {
                    if(ml_link_id.get(index)==sr.getLinkIn())
                    {
                        String [] string_array;
                        if(sr.getLinkOut()==fr_link_id.get(index)){
                            string_array = sr.getContent().split(",");
                            if(enforce_constant_splits){
                                fr_split.add(Double.parseDouble(string_array[0]));
                            }
                            else{
                                for(String str : string_array ) {
                                    fr_split.add(Double.parseDouble(str));
                                }
                            }
                        }
                        else {
                            if(index<ml_link_id.size()-1 && sr.getLinkOut()==ml_link_id.get(index+1)) {
                                string_array = sr.getContent().split(",");
                                if (enforce_constant_splits) {
                                    ml_split.add(Double.parseDouble(string_array[0]));
                                } else {
                                    for (String str : string_array)
                                        ml_split.add(Double.parseDouble(str));
                                }
                            }
                            else
                                throw new Exception("ERROR!");
                        }
                    }
                    else {
                        throw new Exception("Do not define splits for onramp inputs!");
                    }
                }
                if(fr_split.isEmpty() && !ml_split.isEmpty())
                    for(Double d : ml_split)
                        fr_split.add(1-d);
                segments.get(index).set_split_ratios(fr_split,srp.getDt());
            }
        }
    }

    ///////////////////////////////////////////////////////////////////
    // print
    ///////////////////////////////////////////////////////////////////

    public String as_table(double dt) {
        String str = "i\tml_link_id\tor_link_id\tfr_link_id\tfr_node_id\tml_link_length\tvf*dt\tw*dt\tf_max*dt\tn_max\tno\tlo\tis_metered\tl_max\tr_max\n";
        for(int i=0;i<segments.size();i++)
            str += i+"\t"+segments.get(i).as_row(dt)+"\n";
        return str;
    }

    @Override
    public String toString() {
        String str = "";
        for(int i=0;i<segments.size();i++)
            str += segments.get(i).toString()+"\n";
//            str = str.concat(String.format("%d) ----------------------------\n%s",i,segments.get(i).toString()));
        return str;
    }
}
