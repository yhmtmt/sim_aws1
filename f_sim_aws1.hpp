#ifndef F_SIM_AWS1_HPP
#define F_SIM_AWS1_HPP
// Copyright(c) 2017-2020 Yohei Matsumoto, All right reserved. 

// f_sim_aws1.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sim_aws1.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sim_aws1.hpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "filter_base.hpp"
#include "ch_state.hpp"
#include "ch_aws1_ctrl.hpp"
#include "ch_nmea.hpp"

#include "c_model.hpp"

//////////////////////////////////////////////////////// f_sim_aws1
class f_sim_aws1 : public f_base
{
protected:
  // simulation models
  c_model_rudder_ctrl mrctrl; // rudder control model
  c_model_engine_ctrl mectrl; // engine control model
  c_model_outboard_force mobf, mobfp, mobfb; // engine force model, later is for the planing mode and astern mode
  c_model_3dof m3dof, m3dofp, m3dofb;         // kinetic model, later is for the planing mode and astern mode
  double vplane; // planing velocity
  
  // input channels
  ch_state * m_state;
  ch_eng_state * m_engstate;
  
  unsigned char eng_max, eng_nuf, eng_nut, eng_nub, eng_min,
    rud_max, rud_nut, rud_min;
  
  ch_ctrl_data * m_ch_ctrl_out;         // (ui<-autopilot<-sim)
  ch_ctrl_data * m_ch_ctrl_in;          // (ui->autopilot->sim)
  unsigned char buf[64];
  unsigned int buf_len;
  flatbuffers::FlatBufferBuilder builder;
  Control::Config config;
  
  // output channels
  ch_state * m_state_sim;
  ch_eng_state * m_engstate_sim;

  ch_nmea * m_gps_nmea;
  float cycle_gps_report_sec;
  unsigned int cycle_gps_report;
  long long tgps_report;
  char nmea_buf[84];
  c_gga gga_enc;
  c_vtg vtg_enc;
  c_psat_hpr psat_hpr_enc;
  ch_n2k_data * m_eng_n2k_data;
  float cycle_eng_report_sec;
  unsigned int cycle_eng_report;
  long long teng_report;
  
  struct s_state_vector
  {
    long long t;
    double lat, lon, xe, ye, ze, roll, pitch, yaw, cog, sog, ryaw;		
    double Rwrld[9];
    float eng, rud, rev, fuel; 
    float thro_pos, gear_pos, rud_pos, rud_slack;
    
  s_state_vector(const long long & _t,
		 const double & _lat, const double & _lon,
		 const double & _roll, const double & _pitch,
		 const double & _yaw,
		 const double & _cog, const double & _sog,
		 const float & _eng, const float & _rud,
		 const float & _rev, const float & _fuel) :
    t(_t), lat(_lat), lon(_lon),
      roll(_roll), pitch(_pitch), yaw(_yaw), cog(_cog), sog(_sog),
      eng(_eng), rud(_rud), rev(_rev), fuel(_fuel), ryaw(0)
    {
      update_coordinates();
    }
    
  s_state_vector() :lat(135.f), lon(35.f),
      roll(0.f), pitch(0.f), yaw(0.f), cog(0.f), sog(0.f),
      eng(127.0f), rud(127.0f), rev(700.f), fuel(0.1f),
      thro_pos(0.f), gear_pos(0.f),
      rud_pos(0.f), rud_slack(0.f)
    {
      update_coordinates();
    }
    
    void update_coordinates() {
      blhtoecef(lat, lon, 0, xe, ye, ze);
      getwrldrot(lat, lon, Rwrld);			
    }

    void print()
    {
      cout << " rud:" << rud << " rud_pos:" << rud_pos
	   << " rud_slack:" << rud_slack;
      cout << " eng: " << eng << " gear_pos:" << gear_pos
	   << " thro_pos: " << thro_pos;
      cout << " rev:" << rev;
      cout << " yaw:" << yaw;
      cout << " cog:" << cog;
      cout << " sog:" << sog;
    }
  };

  float m_int_smpl_sec;
  unsigned int m_int_smpl; // sampling interval (m_int_smpl_sec * 10e7)
  unsigned int m_iv_head;
  unsigned int m_wismpl; // inputs past m_wismpl * m_int_smpl secs are hold 
  vector<s_state_vector> m_input_vectors; // time sequence of  input vectors
  unsigned int m_wosmpl; // outputs next m_wismpl * m_int_smpl secs are calculated
  vector<s_state_vector> m_output_vectors; // time sequence of output vectors
  
  s_state_vector m_sv_init, m_sv_cur;
  void init_input_sample();
  void update_input_sample();
  void init_output_sample();
  void update_output_sample(const long long & tcur);
  
  // time of previous sampling 
  long long m_tprev;
  
  void get_inst();
  void set_stat();
  void set_input_state_vector(const long long & tcur);
  void set_output_state_vector(const long long & tcur);
  
  void simulate(const long long tcur, const int iosv);
  void simulate_rudder(const float rud, const float rud_pos, float & rud_pos_next);
  void simulate_engine(const float eng, const float eng_pos, const float gear_pos, float & eng_pos_next, float & gear_pos_next);
  
  bool m_bcsv_out;
  char m_fcsv_out[1024];
  ofstream m_fcsv;
  void save_csv(const long long tcur);

  bool bupdate_model_params;
  void update_model_params();
 public:
  f_sim_aws1(const char * name);

  void register_model_params(c_model_base & mdl)
  {
    int n = mdl.get_num_params();
    for (int ipar = 0; ipar < n; ipar++){
      register_fpar(mdl.get_str_param(ipar), mdl.get_param(ipar),
			    mdl.get_str_param_exp(ipar));
    }
  }
  
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif
