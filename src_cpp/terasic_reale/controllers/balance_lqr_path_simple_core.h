#ifndef TERASIC_BALANCE_LQR_PATH_SIMPLE_CORE_H
#define TERASIC_BALANCE_LQR_PATH_SIMPLE_CORE_H
#include "balance_lqr_path_core.h"

// Timed references only. No arrival gates, tracking-error pauses or retries.
// At60s: hold0..3, forward3..13, hold13..15, turn15..45,
// hold45..47, forward47..57, hold57..60, then active balance indefinitely.
class LqrSimplePath {
    LqrPathModel model_;
    double duration_,leg_seconds_,turn_seconds_,initial_hold_,between_hold_,distance_,turn_,first_origin_,turn_position_,second_origin_;
    double last_s_,last_yaw_,world_x_,world_y_;
    bool initialized_,first_started_,turn_started_,second_started_;
    bool continuous_reference_;
public:
    LqrSimplePath(double duration,double distance,double turn,const LqrPathModel &model,double leg_seconds=0,double turn_seconds=0,
                  double initial_hold=3,double between_hold=2,double final_hold=3,bool continuous_reference=false)
      :model_(model),duration_(duration),leg_seconds_(leg_seconds>0 ? leg_seconds : (duration-initial_hold-2*between_hold-final_hold)*.2),
       turn_seconds_(turn_seconds>0 ? turn_seconds : (duration-initial_hold-2*between_hold-final_hold)*.6),initial_hold_(initial_hold),between_hold_(between_hold),distance_(distance),turn_(turn),
       first_origin_(0),turn_position_(0),second_origin_(0),last_s_(0),last_yaw_(0),
       world_x_(0),world_y_(0),initialized_(false),first_started_(false),
       turn_started_(false),second_started_(false),continuous_reference_(continuous_reference) {}
    double first_start()const{return initial_hold_;}
    double leg_duration()const{return leg_seconds_;}
    double turn_start()const{return initial_hold_+leg_duration()+between_hold_;}
    double turn_duration()const{return turn_seconds_;}
    double second_start()const{return turn_start()+turn_duration()+between_hold_;}
    bool schedule_finished(double time)const{return time>=duration_;}
    double world_x()const{return world_x_;}
    double world_y()const{return world_y_;}
    LqrPathReference update(double time,const LqrMimoState &s){
        if(initialized_){
            const double ds=s.x[0]-last_s_,yaw=.5*(last_yaw_+s.x[2]);
            world_x_-=ds*std::sin(yaw);world_y_+=ds*std::cos(yaw);
        }
        initialized_=true;last_s_=s.x[0];last_yaw_=s.x[2];
        if(time>=initial_hold_ && !first_started_){first_origin_=s.x[0];first_started_=true;}
        if(time>=turn_start() && !turn_started_){turn_position_=s.x[0];turn_started_=true;}
        if(time>=second_start() && !second_started_){second_origin_=s.x[0];second_started_=true;}
        LqrPathReference r;
        for(int i=0;i<6;++i)r.x[i]=0;
        const double first_end=initial_hold_+leg_duration(),turn_end=turn_start()+turn_duration();
        const double second_end=second_start()+leg_duration();
        r.phase=time<initial_hold_ ? 0 : (time<first_end ? 1 : (time<turn_start() ? 2 :
            (time<turn_end ? 3 : (time<second_start() ? 4 : (time<second_end ? 5 : 6)))));
        LqrPathProfile leg={0,0,0,0},yaw={0,0,0,0};
        if(time<turn_start()){
            leg=lqr_path_profile(time,initial_hold_,leg_duration(),distance_);
            r.x[0]=time<initial_hold_ ? 0 : first_origin_+leg.s;
        }else if(time<second_start())r.x[0]=turn_position_;
        else{
            leg=lqr_path_profile(time,second_start(),leg_duration(),distance_);
            r.x[0]=second_origin_+leg.s;
        }
        // Preserve tracking error across phases in the initial odometry frame.
        if(continuous_reference_)
            r.x[0]=lqr_path_profile(time,initial_hold_,leg_duration(),distance_).s+
                   lqr_path_profile(time,second_start(),leg_duration(),distance_).s;
        yaw=lqr_path_profile(time,turn_start(),turn_duration(),turn_);
        r.x[1]=model_.pitch_v*leg.v+model_.pitch_a*leg.a;
        r.x[2]=yaw.s;r.x[3]=leg.v;
        r.x[4]=model_.pitch_v*leg.a+model_.pitch_a*leg.j;r.x[5]=yaw.v;
        const double common=100*(model_.common_v*leg.v+model_.common_a*leg.a);
        const double differential=100*(model_.diff_w*yaw.v+model_.diff_alpha*yaw.a);
        r.ff[0]=common-differential;r.ff[1]=common+differential;
        return r;
    }
    // Ideal drawing in the initial frame, distinct from measured odometry.
    double planned_y(double time)const{
        return lqr_path_profile(time,initial_hold_,leg_duration(),distance_).s-
            lqr_path_profile(time,second_start(),leg_duration(),distance_).s;
    }
};
#endif
