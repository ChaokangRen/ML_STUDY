```C++
static ErrorType PropagateOnce(StateTransformer& stf,Vehicle &ego_vhicle,Vehicle &leading_vehicle,decimal_t &dt,Param &param,State *desired_state){
    State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.state();
    auto sim_param = param;

    //1. 计算方向盘转角
    bool steer_calculation_failed = false;
    FrenetState current_fs;
    if(stf.GetFrenetStateFromState(current_state,&current_fs) != kSuccess || current_fs.vec_s[1] < -kEPS){
        steer_calculation_failed = true;
    }

    decimal_t steer,velocity;
    if(!steer_calcuation_failed){
        decimal_t approx_lookhead_dist = std::min(std::max(param.steer_control_min_lookahead_dist,current_state.velocity *param.steer_control_gain),param.steer_control_max_lookahead_dist);
        if(CalcualateSteer(stf,current_state,current_fs,wheelbase_len,Vec2f(apporx_lookahead_dist,0.0),&steer) != kSuccess){
            steer_calculation_failed = true;
        }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;

    if(param.auto_decelerate_if_lat_failed && steer_calculation_failed){
        sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0,sim_vel);

    //2. 计算速度
    FrenetState leading_fs;
    //如果车辆id异常或者前车在frenet坐标系中的位置异常
    if(leading_vehicle.id() == kInvalidAgentId || stf.GetFrenetStateFromState(leading_vehicle.state(),&leading_fs) != kSuccess){
        //不带前车计算期望速度
        CalculateVeclocityUsingIdm(current_state.velocity,dt,sim_param,&velocity);
    }else{
        //带前车计算期望速度
        //对于 IDM，减去车辆长度即可得到本车与领先车辆之间的“净”距离。
        decimal_t eqv_vehicle_len;
        GetIdmEquvalentVehicleLength(stf,ego_vehicle,leanding_vehicle,leading_fs,&eqv_vehicle_len);
        sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

        CalculateVelocityUsingIdm(current_fs.vec_s[0],current_state.velocity,leading_fs.vec_s[0],leading_vehicle.state(),dt,sim_param,&velocity);
    }

    //3. 获取期望的车辆状态
    CalculateDesiredState(current_state,steer,velocity,wheelbase_len,dt,sim_param,desired_state);
    return kSuccess;
}


```