类： SemanticMapManager
成员变量：
1. time_stamp:应该是时间戳
2. pred_time = 5.0:预测总时间
3. pred_step_ = 0.2: 预测迭代步长
4. nearest_lane_range_ = 1.5:有点像从车道中线到车道线的距离
5. lane_range_ = 10.0：应该是自车周围可见的车道横向范围
6. max_distance_to_lane = 2.0:距离车道线的最大距离
7. bool has_fast_lut = false:暂时看不出来具体用途
8. local_lanes:局部车道的集合
9. local_to_segment_lut:
10. segmengt_to_local_lut:
11. local_lane_length_forward = 250:前向最远可视车道线距离
12. local_lane_length_backward = 150:后向最远可视车道线距离
13. ego_id :自车的id
14. agent_config_path:配置文件路径
15. use_right_hand_axis:使用右手准则
16. ego_vehicle:自车的车辆实例
17. obstacle_map:障碍物map
18. `std::set<std::array<decimal_t,2>> obstacle_grids`:障碍物网格位置
19. VehicleSet:surrounding_vehicles:周围车辆类集合
20. semantic_surrounding_vehicles:周围车辆可能的语义集合
21. key_vehicle:关键车辆
22. semantic_key_vehicles:关键车辆的语义
23. `vector<int> key_vehicle_ids`:关键车辆的id
24. `vector<int>uncertain_vehicle_ids`:不确定车辆的id
25. `LaneNet whole_lane_net`:整个路网
26. `LaneNet surrounding_lane_net` 自车周围路网
27. `SemanticLaneSet semantic_lane_set`:路网语义
28. `SemanticBehivior ego_behavior`:语义行为
29. `unordered_map<int,vec_E<State>> openloop_pred_trajs`:开环预测轨迹
30. `TicToc global_timer`:全局时间
31. `TrafficSignalManager: traffic_singal_manager`:交通灯manager
32. `ConfigLoader *p_config_loader`:
33. `RssChecker rss_checker`rss安全检测
34. `is_simple_lane_structure` 是否是简单道路结构

成员函数：
1. ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,bool *res):
该函数从函数名称来看主要是用于碰撞检测，接受一组坐标并返回结果。  
函数实现主要通过obstacle_map_.CheckIfEqualUsingGlobalPosition(p,GridMap2D::OCCUPIED,res),从该函数来看，应该是检测每一个障碍物是否与该点碰撞来确定。进一步细节需要跳到obstacle_map的实现去看。
2. ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,ObstacleMapType *res);

```
3. MobilRuleBasedBehaviorPrediction():
输入:vheicle、nearby_vehicles  
输出:ProbDistOfLatBehaviors *res(横向行为的概率)

1. 构建 behaviors{keep,left,right}
2. 遍历behaviors:
3. GetRefLaneForStateByBehavior(vehicle.state,vector<int>(),behavior,lane_radius,lane_radius,false,&ref_lane):根据当前行为来获取最近的ref lane
```

```C++
成员函数
GetRefLaneForStateByBehavior(State,navi_path,behavior,max_forward_len,max_back_len,is_high_quality,*lane):
1. 构造三自由度状态: Vec3f state_3dof(state.vec_position(0),state.vec_position(1),state.angle);
2. 声明: int current_lane_id;dicimal_t distance_to_lane,arc_len;
3. if(GetNearestLaneIdUsingState(state_3dof,navi_path,&current_lane_id,&distance_to_lane,&arc_len) != kSuccess){
    return kWongStatus.
}
//如果函数GetNearestLaneIdUsingState()返回值不是success,则返回wrong。
//该函数意思为获取当前距离和角度最近的那条车道的相关数据:id,dist,arc_len
4. if(distance_to_lane >max_distance_to_lane) return kWongStatus;
5. int target_lane_id
6. if(GetTargetLaneId(current_lane_id,behavior,&target_lane_id)!=kSuccess){
    return kWrongStatus;
    //获取当前目标车道的id
}
7. if(agent_config_info_.enable_fast_lane_lut && has_fast_lut_){
    if(segment_to_local_lut_.end() != segment_to_local_lut_.find(target_lane_id)){
        //我们仅选择几条候选车道的第一条车道
        int id = *segment_to_local_lut_.at(target_lane_id).begin();
        *lane = local_lanes.at(id);
        return kSuccess;
    }
8. //reflane的长度应该符合最大速度和最大前面仿真时间，当前设置为30m/s和7.5s
   vec_Vecf<2> samples;
   if(GetLocalLaneSamplesByState(state,target_lane_id,navi_path,max_forward_len,max_back_len,&samples) != kSuccess){
    return kWrongStatus;
    //通过当前状态获取局部车道的采样
   }
9. if(kSuccess != GetLaneBySampledPoints(samples,is_high_quality,lane)){
    reuturn kWrongStatus;
    }
10. return kSuccess;
}
```

```C++
ErrorType SemanticMapManager::GetLocalSamplesByState(state,lane_id,navi_path,max_reflane_dist,max_backward_dist,vec_Vecf<2> *samples)const{
1. if(semantic_lane_set_.semantic_lanes.count(lane_id) == 0){
        return kWrongStatus;
    }
2. decimal_t arclen = 0.0;
3. Lane target_lane = semantic_lane_set.semantic_lanes.at(lane_id).lane;
4. target_lane.GetArcLengthByVecPosition(state.vec_position,&arclen);
//通过当前状态位置获取lane上离该点最近的s
5. decimal_t accum_dist_backward = 0,vector<int> ids_back;
6. if(arclen < max_backward_dist){
    accum_dist_backward += arclen;
    int id_tmp = lane_id;
    while(accum_dist_backward <max_backward_dist){
        vector<int> father_ids = semantic_lane_set_.semantic_lanes.at(id_tmp).father_id;
    }
}
}
```