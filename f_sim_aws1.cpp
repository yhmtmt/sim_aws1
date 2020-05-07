// Copyright(c) 2017-2020 Yohei Matsumoto, All right reserved. 

// f_sim_aws1.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sim_aws1.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sim_aws1.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <list>
#include <map>
using namespace std;

#include "aws_coord.hpp"

#include "f_sim_aws1.hpp"

DEFINE_FILTER(f_sim_aws1)

/////////////////////////////////////////////////////////////////////////// f_sim_aws1 members

f_sim_aws1::f_sim_aws1(const char * name) :
  f_base(name), vplane(10.0),
  m_state(NULL), 
  m_ch_ctrl_out(nullptr), m_ch_ctrl_in(nullptr),
  m_state_sim(nullptr), m_engstate_sim(nullptr),
  m_tprev(0), m_bcsv_out(false), m_int_smpl_sec(0.03333333),
  m_wismpl(100), m_wosmpl(1), bupdate_model_params(true)
{
	// input channels for simulation results
  register_fpar("ch_state", (ch_base**)&m_state,
		typeid(ch_state).name(), "State channel");
  register_fpar("ch_engstate", (ch_base**)&m_engstate,
		typeid(ch_eng_state).name(), "Engine Status channel");

  // output channels for simulation results
  register_fpar("ch_state_sim", (ch_base**)&m_state_sim,
		typeid(ch_state).name(), "State channel");
  register_fpar("ch_engstate_sim", (ch_base**)&m_engstate_sim,
		typeid(ch_eng_state).name(), "Engine Status channel");  
  register_fpar("ch_ctrl_out", (ch_base**)&m_ch_ctrl_out,
		typeid(ch_ctrl_data).name(), "Control data out (to autopilot)");
  register_fpar("ch_ctrl_in", (ch_base**)&m_ch_ctrl_in,
		typeid(ch_ctrl_data).name(), "Control data in (from autopilot)");
  register_fpar("int_smpl", &m_int_smpl_sec, "Sampling interval in second");
  register_fpar("wismpl", &m_wismpl, "Width of input sampling window");
  register_fpar("wosmpl", &m_wosmpl, "Width of output sampling window");

  // initial values of input vector
  register_fpar("lat0", &m_sv_init.lat, "Initial Latitude(deg)");
  register_fpar("lon0", &m_sv_init.lon, "Initial Longitude(deg)");
  register_fpar("roll0", &m_sv_init.roll, "Initial Roll(deg)");
  register_fpar("pitch0", &m_sv_init.pitch, "Initial Pitch(deg)");
  register_fpar("yaw0", &m_sv_init.yaw, "Initial Yaw(deg)");
  register_fpar("cog0", &m_sv_init.cog, "Initial Course over ground(deg)");
  register_fpar("sog0", &m_sv_init.sog, "Initial Speed over ground(kts)");
  register_fpar("eng0", &m_sv_init.eng, "Initial Engine control value");
  register_fpar("rud0", &m_sv_init.rud, "Initial Rudder control value");
  register_fpar("rev0", &m_sv_init.rev, "Initial Engine rev (RPM).");
  register_fpar("fuel0", &m_sv_init.fuel, "Initial fuel flow rate (L/h)");

  // Each control points of the main engine output.
  register_fpar("eng_max", &eng_max,
		"Maximum control value for AWS1's main engine.");
  register_fpar("eng_nuf", &eng_nuf,
		"Nutral to Forward control value for AWS1's main engine.");
  register_fpar("eng_nut", &eng_nut,
		"Nutral control value for AWS1's main engine.");
  register_fpar("eng_nub", &eng_nub,
		"Nutral to Backward control value for AWS1's main engine.");
  register_fpar("eng_min", &eng_min,
		"Minimum control value for AWS1's main engine.");
  
  // Each controll points of the rudder output.
  register_fpar("rud_max", &rud_max,
		"Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &rud_nut,
		"Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &rud_min,
		"Minimum control value for AWS1's rudder.");
  
  m_fcsv_out[0] = '\0';
  register_fpar("fcsv", m_fcsv_out, 1024, "CSV output file.");
  register_fpar("vplane", &vplane, "Planing Velocity in kts");
  register_fpar("update_model_params", &bupdate_model_params,
		"Update model params.");
  mrctrl.alloc_param();
  register_model_params(mrctrl);
  mectrl.alloc_param();
  register_model_params(mectrl);
  mobf.alloc_param(0);
  register_model_params(mobf);
  mobfp.alloc_param(1);
  register_model_params(mobfp);
  mobfb.alloc_param(2);
  register_model_params(mobfb);
  m3dof.alloc_param(0);
  register_model_params(m3dof);
  m3dofp.alloc_param(1);
  register_model_params(m3dofp);
  m3dofb.alloc_param(2);
  register_model_params(m3dofb);
}

bool f_sim_aws1::init_run()
{
  m_int_smpl = (unsigned int)(m_int_smpl_sec * SEC);
  m_sv_cur.t = 0;
  
  init_input_sample();
  init_output_sample();
  
  if (m_fcsv_out[0]) {
    m_fcsv.open(m_fcsv_out, ios::binary);
    if (!m_fcsv.is_open()) {
      cerr << "Failed to open file " << m_fcsv_out << endl;
      return false;
    }
    
    // first row of the csv file
    m_fcsv <<
      "t,lat_o,lon_o,xe_o,ye_o,ze_o,roll_o,pitch_o,yaw_o,sog_o,cog_o,eng_o,rud_o,rev_o,fuel_o,"
	   << "thro,gear,rud,"
	   <<"lat_i,lon_i,xe_i,ye_i,ze_i,roll_i,pitch_i,yaw_i,sog_i,cog_i,eng_i,rud_i,rev_i,fuel_i," 
	   << endl;
    m_fcsv.precision(3);
  }

  update_model_params();
  return true;
}

void f_sim_aws1::update_model_params()
{
  mrctrl.init();
  mectrl.init();
  mobf.init();
  mobfp.init();
  m3dof.init();
  m3dofp.init();
  m3dofb.init();
  bupdate_model_params = false;
}


void f_sim_aws1::destroy_run()
{
  if (m_fcsv.is_open()) {
    m_fcsv.close();
  }
}

void f_sim_aws1::get_inst()
{
  if(m_ch_ctrl_in){
    while(1){
      m_ch_ctrl_in->pop(buf, buf_len);
      if(buf_len == 0)
	break;

      auto data = Control::GetData(buf);
      switch(data->payload_type()){
      case Control::Payload_Engine:
	m_sv_cur.eng = (float)data->payload_as_Engine()->value();
	if(m_ch_ctrl_out) m_ch_ctrl_out->push(buf, buf_len);
	break;
      case Control::Payload_Rudder:
	m_sv_cur.rud = (float)data->payload_as_Rudder()->value();
	if(m_ch_ctrl_out) m_ch_ctrl_out->push(buf, buf_len);	
	break;
      case Control::Payload_Config:
	config = *data->payload_as_Config();
	break;
      }
    }
  }
  
  m_sv_cur.thro_pos = m_output_vectors[0].thro_pos;
  m_sv_cur.gear_pos = m_output_vectors[0].gear_pos;
  m_sv_cur.rud_pos = m_output_vectors[0].rud_pos;
  m_sv_cur.rud_slack = m_output_vectors[0].rud_slack;
}

void f_sim_aws1::set_stat()
{
  if(m_ch_ctrl_out &&
     (config.engine_max() != eng_max ||
      config.engine_min() != eng_min ||
      config.engine_nutral() != eng_nut ||
      config.engine_forward() != eng_nuf ||
      config.engine_backward() != eng_nub ||
      config.rudder_max() != rud_max ||
      config.rudder_min() != rud_min ||
      config.rudder_mid() != rud_nut)){
    builder.Clear();
    auto payload = builder.CreateStruct(Control::Config(eng_max, eng_nuf,
							eng_nut, eng_nub,
							eng_min, rud_max,
							rud_nut, rud_min));
    auto data = CreateData(builder, get_time(),
			   Control::Payload_Config, payload.Union());
    builder.Finish(data);
    m_ch_ctrl_out->push(builder.GetBufferPointer(), builder.GetSize());
    
  }  
}

void f_sim_aws1::set_input_state_vector(const long long & tcur)
{
  long long tprev = m_sv_cur.t;
  m_sv_cur.t = tcur;
  if (m_state){
    long long t = 0;
    float roll, pitch, yaw, cog, sog, ryaw;
    double lat, lon;
    
    m_state->get_attitude(t, roll, pitch, yaw);
    m_state->get_position(t, lat, lon);
    m_state->get_velocity(t, cog, sog);
    m_sv_cur.roll = roll * (PI / 180.f);
    m_sv_cur.pitch = pitch * (PI / 180.f);
    m_sv_cur.cog = cog * (PI / 180.f);
    m_sv_cur.sog = sog;
    m_sv_cur.lat = lat * (PI / 180.f);
    m_sv_cur.lon = lon * (PI / 180.f);
    m_sv_cur.update_coordinates();
    if(tprev != 0){ // calculate yaw rate
      yaw *= (PI / 180.f);
      float yaw_prev = m_sv_cur.yaw;
      double dyaw = normalize_angle_rad(yaw - yaw_prev);
      m_sv_cur.ryaw = (double)(dyaw /((double)m_int_smpl / (double)SEC));
      m_sv_cur.yaw = yaw;
    }else{
      m_sv_cur.yaw = yaw * (PI / 180.f);
      m_sv_cur.ryaw = 0.f;
    }	
  }

  if (m_engstate)
  {
    long long t = 0;
    unsigned char trim = 0;
    int poil = 0;
    float toil = 0.0f;
    float temp = 0.0f;
    float valt = 0.0f;
    float frate = 0.0f;
    unsigned int teng = 0;
    int pclnt = 0;
    int pfl = 0;
    unsigned char ld = 0;
    unsigned char tq = 0;
    NMEA2000::EngineStatus1 steng1 =
      (NMEA2000::EngineStatus1)(NMEA2000::EngineStatus1_MAX + 1);
    NMEA2000::EngineStatus2 steng2 =
      (NMEA2000::EngineStatus2)(NMEA2000::EngineStatus2_MAX + 1);
    
    m_engstate->get_rapid(t, m_sv_cur.rev, trim);
    m_engstate->get_dynamic(t, poil, toil, temp, valt, frate,
			    teng, pclnt, pfl, steng1, steng2, ld, tq);
    m_sv_cur.fuel = frate;
  }

  get_inst(); // select and load correct control values to msv_cur.rud and m_sv_cur.eng
}


void f_sim_aws1::set_output_state_vector()
{
  s_state_vector sv = m_output_vectors[0];
  if (m_engstate_sim)
    {
      // output simulated engine state
      long long t = 0;
      unsigned char trim = 0;
      int poil = 0;
      float toil = 0.0f;
      float temp = 0.0f;
      float valt = 0.0f;
      float frate = 0.0f;
      float rev = 0.0f;
      unsigned int teng = 0;
      int pclnt = 0;
      int pfl = 0;
      unsigned char ld = 0;
      unsigned char tq = 0;
      NMEA2000::EngineStatus1 steng1 =
	(NMEA2000::EngineStatus1)(NMEA2000::EngineStatus1_MAX + 1);
      NMEA2000::EngineStatus2 steng2 =
	(NMEA2000::EngineStatus2)(NMEA2000::EngineStatus2_MAX + 1);
      
      // overwrite only rev 
      m_engstate->get_rapid(t, rev, trim); 
            m_engstate_sim->set_rapid(sv.t, sv.rev, trim);
      
      // overwrite only frate
      m_engstate->get_dynamic(t, poil, toil, temp, valt, frate,
			      teng, pclnt, pfl, steng1, steng2, ld, tq);
      m_engstate_sim->set_dynamic(sv.t, poil, toil, temp, valt, sv.fuel,
      				  teng, pclnt, pfl, steng1, steng2, ld, tq);
    }
  
  if (m_state_sim)
    {
      float alt = 0.f, galt = 0.f;
      //output simulated lat, lon, roll, pitch, yaw, cog, sog;
      m_state_sim->set_attitude(sv.t, sv.roll * (180.f / PI),
				sv.pitch * (180.f / PI), sv.yaw * (180.f / PI));
      m_state_sim->set_position(sv.t, sv.lat * (180.f / PI),
				sv.lon * (180.f / PI));
      m_state_sim->set_velocity(sv.t, sv.cog * (180.f / PI), sv.sog);
    }

  set_stat();
}

void f_sim_aws1::init_input_sample()
{
  m_sv_init.update_coordinates();
  m_input_vectors.resize(m_wismpl);
  m_iv_head = m_wismpl - 1;
  long long t = get_time();
  
  for (int iv = m_iv_head, nv = 0; nv < m_wismpl; nv++) {
    m_input_vectors[iv] = m_sv_init;
    m_input_vectors[iv].t = t - nv * m_int_smpl;
    if (iv == 0)
      iv = m_wismpl - 1;
    else
      iv = iv - 1;
  }
}

void f_sim_aws1::update_input_sample()
{
  m_iv_head++;
  if (m_iv_head == m_wismpl)
    m_iv_head = 0;
  
  m_input_vectors[m_iv_head] = m_sv_cur;
}

void f_sim_aws1::init_output_sample()
{
  m_sv_init.update_coordinates();
  m_output_vectors.resize(m_wosmpl);
  long long t = get_time();
  for (int ov = 0; ov < m_wosmpl; ov++) {
    m_output_vectors[ov] = m_sv_init;
    m_output_vectors[ov].t = t + ov * m_int_smpl;
  }
}

void f_sim_aws1::update_output_sample(const long long & tcur)
{
  double dt = (double) m_int_smpl / (double) SEC;
  for (int iosv = 0; iosv < m_wosmpl; iosv++) {		
    s_state_vector & stprev =
      (iosv == 0 ? m_input_vectors[m_iv_head] : m_output_vectors[iosv-1]);
    s_state_vector & stcur = m_output_vectors[iosv];
    stcur.t = tcur + iosv * m_int_smpl;
    
    // simulate actuator and pump  
    double v[3];
    double f[3];
    double phi = (stprev.cog - stprev.yaw);
    double th = stprev.cog;
    
    double sog_ms = stprev.sog * (1852. / 3600.);
    double dx = sog_ms * dt * sin(th),
      dy = sog_ms * dt * cos(th); //next position in enu coordinate
    double alt = 0.;
    
    wrldtoecef(stprev.Rwrld, stprev.xe, stprev.ye, stprev.ze, dx, dy, 0.,
	       stcur.xe, stcur.ye, stcur.ze);
    eceftoblh(stcur.xe, stcur.ye, stcur.ze, stcur.lat, stcur.lon, alt);
    
    v[0] = sog_ms * cos(phi);
    v[1] = sog_ms * sin(phi);
    v[2] = stprev.ryaw;
    mrctrl.update(stprev.rud, stprev.rud_pos, stprev.rud_slack, dt,
		  stcur.rud_pos, stcur.rud_slack);
    mectrl.update(stprev.eng, stprev.gear_pos, stprev.thro_pos,
		  stprev.rev, dt,
		  stcur.gear_pos, stcur.thro_pos, stcur.rev);

    if(v[0] < 0){
      // astern model
      mobfb.update((stprev.rud_pos - stprev.rud_slack),
		  stprev.gear_pos, stprev.thro_pos,
		  stprev.rev, v, f);
      m3dofb.update(v, f, dt, v);
    }else if(v[0] < vplane){
      // displacement model
      mobf.update((stprev.rud_pos - stprev.rud_slack),
		  stprev.gear_pos, stprev.thro_pos,
		  stprev.rev, v, f);
      m3dof.update(v, f, dt, v);
    }
    else{
      // planing model
      mobfp.update((stprev.rud_pos - stprev.rud_slack),
		   stprev.gear_pos, stprev.thro_pos,
		   stprev.rev, v, f);
      m3dofp.update(v, f, dt, v);
    }

    phi = atan2(v[1], v[0]);
    stcur.yaw += v[2] * dt;
    if(stcur.yaw > PI)
      stcur.yaw -= 2 * PI;
    else if(stcur.yaw < -PI)
      stcur.yaw += 2 * PI;      
    
    stcur.ryaw = v[2];
    stcur.cog = stcur.yaw + phi;
    if(stcur.cog < 0)
      stcur.cog += 2 * PI;
    else if(stcur.cog > 2 *PI)
      stcur.cog -= 2 * PI;
    stcur.sog = sqrt(v[0] * v[0] + v[1] * v[1]) * (3600. / 1852.);
  }
}

void f_sim_aws1::save_csv(const long long tcur)
{
  s_state_vector & svo = m_output_vectors[0];
  s_state_vector & svi = m_input_vectors[m_iv_head];
  m_fcsv << tcur << ",";
  
  m_fcsv.precision(8);
  m_fcsv <<
    svo.lat * (180.f/PI) << "," <<
    svo.lon * (180.f/PI) << "," <<
    svo.xe << "," <<
    svo.ye << "," <<
    svo.ze << ",";
  
  m_fcsv.precision(3);
  m_fcsv <<
    svo.roll * (180.f/PI)<< "," <<
    svo.pitch * (180.f/PI)<< "," <<
    svo.yaw  * (180.f/PI)<< "," <<
    svo.sog << "," <<
    svo.cog  * (180.f/PI)<< "," <<
    svo.eng << "," <<
    svo.rud << "," <<
    svo.rev << "," <<
    svo.fuel << ",";
  m_fcsv <<
    svo.thro_pos << "," <<
    svo.gear_pos << "," <<
    svo.rud_pos << ",";
  
  m_fcsv.precision(8);
  m_fcsv <<
    svi.lat * (180.f/PI) << "," <<
    svi.lon * (180.f/PI) << "," <<
    svi.xe << "," <<
    svi.ye << "," <<
    svi.ze << ",";
  
  m_fcsv.precision(3);
  m_fcsv <<
    svi.roll * (180.f/PI) << "," <<
    svi.pitch * (180.f/PI) << "," <<
    svi.yaw * (180.f/PI) << "," <<
    svi.sog << "," <<
    svi.cog * (180.f/PI) << "," <<
    svi.eng << "," <<
    svi.rud << "," <<
    svi.rev << "," <<
    svi.fuel << ",";

  m_fcsv << endl;
}

bool f_sim_aws1::proc()
{
  if(bupdate_model_params){
    update_model_params();
  }
  
  long long tcur = get_time();
  if (tcur < m_tprev + m_int_smpl)
    return true;
  
  update_output_sample(tcur);
  set_output_state_vector();
  
  set_input_state_vector(tcur);
  update_input_sample();
  
  if (m_fcsv.is_open()) {
    save_csv(tcur);
  }
  
  m_tprev = tcur;
  return true;
}

