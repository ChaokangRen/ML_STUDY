# 类 eudm_planner分析：
相关数据结构：
```C++
enum class LatSimMode{
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
};

struct ForwardSimEgoAgent{
    decimal_t lat_range;

    //udpate on scenario-level
    OnLaneForwardSimulation::Param sim_param;

    LatSimMode seq_lat_mode;
    LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined};
    LateralBehavior seq_lat_behavior;
    bool is_cancel_behavior;
    decimal_t operation_at_seconds{0.0};

    //udate on layer-level
    LongitudinalBehavior lon_behavior{LongitudianlBehavior::kMaintain};
    LateralBehavior lat_behavior{LatralBehavior::kUndefined};

    Lane current_lane;
    StateTransformer current_stf;
    Lane target_lane;
    StateTransformer target_stf;
    Lane longterm_lane;
    StateTransformer longterm_stf;
    
    Vec2i target_gap_ids; //optional,gap ids are fixed in each layer,bug gap is changing in each step

    Vehicle vehicle;
}

struct ForwardSimAgent{
    int id = kInvalidAgentId;
    Vehicle vehicle;

    OnlaneForwardSimulation::Parm sim_param;

    ProbDistOfLatBehaviors lat_probs;
    LateralBehaviors lat_behavior{LateralBehavior::kUndefined};

    Lane lane;
    StateTransformer stf;

    decimal_t lat_range;
}

struct ForwardSimAgentSet{
    std::unordered_map<int,ForwardSimAgent> forward_sim_agents;
}

struct EfficiencyCost{
    decimal_t ego_to_desried_vel = 0.0;
    decimal_t leading_to_desried_vel = 0.0;
    decimal_t ave() const{
        return (ego_to_desired_vel + leading_to_desried_vel) / 2.0;
    }
}

struct SafetyCost{
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const {return (rss + occu_lane) / 2.0;}
}

struct NavigationCost{
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() {return lane_change_preference;}
}

struct CostStructure{
    // associate cost with micro action using this index
    int valid_sample_index_ub;
    //efficiency
    EfficiencyCost efficiency;
    //safety
    SafetyCost safety;
    //navigation
    NavigationCost navigation;

    decimal_t wight = 1.0;
    decimal_t ave() const{
        return (efficiency.ave() + safety.ave() + navigation.ave()) * wight;
    }
}

```
成员变量：
```C++
1. EudmPlannerMapItf* map_itf_{nullptr}; //调用semantic_map的指针
2. DcpTree* dcp_tree_ptr_;
3. Cfg cfg_;
4. LaneChangeInfo lc_info_;
5. decimal_t desired_velocity_{5.0};
6. decimal_t sim_time_total = 0.0;
7. std::set<int> pre_deleted_seq_ids_;
8. int ego_lane_id_{kInvalidLaneId};
9. vector<int> potential_lcl_lane_ids_;
10. vector<int> potential_lcr_lane_ids_;
11. vector<int> potential_lk_lane_ids_;
12. Lane rss_lane_;
13. StateTransformer rss_stf_;
14. RssChecker::RssConfig rss_config_;
15. RssChecker::RssConfig rss_config_strict_as_front_;
16. RssChecker::RssConfig rss_config_strict_as_rear_;

17. OnLaneForwardSimulation::Param ego_sim_param_;
18. OnLaneForwardSimulation::Param agent_sim_param_;

19. decimal_t time_stamp_;
20. int ego_id_;
21. Vehicle ego_vehicle_;

22. int winner_id_ = 0;
23. decimal_t winner_score_ = 0.0;
24. vector<int> sim_res_;
25. vector<int> risky_res_;
26. vector<string> sim_info_;
27. vector<decimal_t> final_cost_;
28. vector<vector<CostStructure>> progress_cost_;
29. vector<CostStructure> tail_cost;
30. vec_E<vec_E<Vehicle>> forward_trajs_;
31. vector<vector<LateralBehavior>> forward_lat_behaviors_;
32. vector<vector<LongitudinalBehavior>> forward_lon_behaviors_;
33. vec_E<std::unordered_map<int,vec_E<Vehicle>>> surround_trajs_;
33. decimal_t time_cost_ = 0.0;

```

core function
```C++
ErrorType EudmPlanner::RunOnce(){
    TicToc timer_runonce;

    // * Get current nearest lane id
    if(!map_itf_){
        return kWrongStatus;
    }
    if(map_itf_->GetEogVehicle(&ego_vehicle_) != kSuccess){
        return KWrongStatus;
    }
    ego_id_ = ego_vehicle_.id();
    time_stamp_ = ego_vehicle_.state().time_stamp;

    int ego_lane_id_by_pos = kInvalidLaneId;
    if(map_itf_ -> GetEgoLaneIdByPosition(std::vector<int>(),&ego_lane_id_by_pos)!=kSuccess){
        return kWrongStatus;
    }
    ego_lane_id_ = ego_lane_id_by_pos;

    decimal_t forward_rss_check_range = 130;
    decimal_t backward_rss_check_range = 130;
    decimal_t forward_lane_len = forward_rss_check_range;
    //对lane keep获取reflane
    if(map_itf_->GetRefLaneForStateByBehavior(ego_vehicle_.state(),std::vector<int>(),LateralBehavior::kLaneKeeping,forward_lane_len,backward_lane_len,false,&rss_lane_)!=kSuccess){
        //
    }

    if(rss_lane.IsValid()){
        rss_stf_ = StateTransformer(rss_lane_);
    }

    pre_deleted_seq_ids_.clear();
    int n_sequence = dcp_tree_ptr_->actrion_script().size();
    for(int i = 0;i < n_sequence;i++){
        auto action_seq = dcp_tree_ptr_->action_script()[i];
        int num_actions = action_seq.size();
        //如果当前这条动作链里面有从左到右或者从右到左的行为变化，则将该条链记录到pre_deleted_seq_ids，这样做应该是不允许这样变化
        for(int j = 0;j < num_actions;j++){
            if((action_seq[j-1].lat == DcpLatAction::kLaneChangeLeft && action_seq[j].lat == DcpLatAction::kLaneChangeRight)||(action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)){
            pre_deleted_seq_ids_ = insert(i);
           }
        }
    }

    TicToc timer;
    if(RunEudm() != kSuccess){
        return kWrongStatus;
    }
}

ErrorType EudmPlanner::RunEudm(){
    //*get relevant information
//1. 获取周围的key vehicles
    SemanticVehicleSet surrounding_semantic_vehicles;
    if(map_itf_->GetKeySemanticVehcles(&surrounding_senmantic_vehicles) != kSuccess){
        return kWrongStatus;
    }
//2. 对周围agents的仿真类实例surronding_fsagnets
    ForwardSimAgentSet surrouding_fsagents;
    GetSurroundingForwardSimAgents(surrounding_segmantic_vehicles,&surrounding_fsagents);

//

    auto action_script = dcp_tree_ptr_->action_script();
    int n_sequence = action_script.size();

    //prepare for multi-threding
    std::vector<std::thread> thread_set(n_sequence);
    PrepareMultiTreadContainers(n_sequence);

    //*threading
    TicToc timer;
    for(int i = 0;i < n_sequence;++i){
        thread_set[i] = std::thread(&EudmPlanner::SimulateActionSequence,this,ego_vehicle_,surrounding_fsagnets,action_script[i],i);
    }
    for(int i = 0;i < n_sequence;++i){
        thread_set[i].join();
    }
}

ErrorType EudmPlanner::GetSurroundingForwardSimAgents(const common::SemanticVehicleSet& surrounding_semantic_vehicles,ForwardSimAgentSet* fsagents)const {
    for(const auto &psv : surrounding_semantic_vehicle.semantic_vehicles){
        ForwardSimAgent fsagent;

        int id = psv.second.vehicle.id();
        fsagent.id = id;
        fsagent.vehicle = psv.second.vechicle;

        fsagnet.sim_param = agent_sim_param_;
        
        State state = psv.second.vehicle.state();

        // * lon
        //如果当前agent的加速 > 0，则设置期望速度为恒定，否则加上加速度的影响
        if(state.acceleration >= 0){
            fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
        }else{
            decimal_t est_vel = std::max(0.0,state.velocity +state.acceleration * sim_time_total);
            fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
        }

        // * lat
        fsagent.lat_probs = psv.second.probs_lat_behaviors;
        fsagent.lat_behavior = psv.second.lat_behavior;

        fsagent.lane = psv.second.lane;
        fsagent.stf = StateTransformer(fsagent.lane);

        // * other
        fsagent.lat_range = cfg.sim().agent().cooperative_lat_range();
        fsagents->forward_sim_agents.insert(std::make_pair(id,fsagent));
    }
}

ErrorType EudmPlanner::SimulateActionSequence(Vehicle& ego_vehicle,ForwardSimAgentSet& surrounding_fsagents,vector<DcpAction>& action_seq,const int &seq_id){
    //1. 如果在pre_deleted_seq_ids里面找到seq_id，那么返回
    if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
        sim_res_[seq_id] = 0;
        sim_info_[seq_id] = std::string("(Pre-deleted)");
        return kWrongStatus;
    }

    int n_sub_thread = 1;

    std::vector<int> sub_sim_res(n_sub_threads);
    std::vector<int> sub_risky_res(n_sub_threads);
    std::vector<std::string> sub_sim_info(n_sub_threads);
    std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
    std::vector<CostStructure> sub_tail_cost(n_sub_threads);
    vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
    std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
        n_sub_threads);
    std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
        n_sub_threads);
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(n_sub_threads);

    SimulateScenario(ego_vehicle,surrounding_fsagents,action_seq,seq_id,0,&sub_sim_res,&sub_risky_res,&sub_sim_info,&sub_progress_cost,&sub_tail_cost,&sub_forward_trajs,&sub_forward_lat_behaviors,&sub_forward_lon_behaviors,&sub_surround_trajs);
}

ErrorType EudmPlanner::SimulateScenerio(Vehicle &ego_vehicle,ForwardSimAgentSet &surrounding_fsagents,vector<DcpAction> &action_seq,const int& seq_id,const int &sub_seq_id,vector<int> *sub_sim_res,,vector<int> *sub_risky_res,vector<string> *sub_sim_info,vector<vector<ConstStructrue>> *sub_progress_cost,vector<CostStructure> * sub_tail_cost,vec_E<vec_E<Vehicle>> *sub_forward_trajs,vector<vector<LateralBehavior>> *sub_forward_lat_behaviors,vector<vector<LongitudinalBehavior>> *sub_forward_lon_behaviors,vec_E<unordered_map<int,vec_E<Vehicle>>> *sub_surround_trajs){

    vec_E<Vehicle> ego_traj_multilayers{ego_vehicle};
    std::unordered_map<int,vec_E<Vehicle>> surround_trajs_multilayers;

    for(const auto &p_fsa:surrounding_fsagnets.forward_sim_agents){
        surrounding_trajs_multilayers.insert(std::pair<int,vec_E<Vehicle>>(p_fsa.first,vec_E<Vehicle>({p_fsa.second.vehicle})));
    }

    vector<LateralBehavior> ego_lat_behavior_multilayers;
    vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
    vector<CostStructure> cost_multilayers;
    std::set<int> risky_ids_multilayers;

    //setup ego longitudinal sim config
    ForwardSimEgoAgent ego_fsagent_this_layer;
    ego_fsagent_this_layer.vehicle = ego_vehicle;
    UpdateSimSetupForScenario(action_seq,&ego_fsagent_this_layer);
}

ErrorType EudmPlanner::UpdateSimSetupForScenario(const vector<DcpAction> &actin_seq,ForwardSimEgoAgent *ego_fsagent) const{
    //Get the type of lateral action sequence
    LateralBehavior seq_lat_behavior;
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    //对动作进行分类
    ClassifyActionSeq(action_seq,&operator_at_seconds,&seq_lat_behavior,&is_cancel_behavior);
}
```