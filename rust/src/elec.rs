/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#![allow(dead_code)]
#![allow(non_camel_case_types)]

use std::os::raw::c_char;
use std::os::raw::c_void;
use std::ffi::CString;
use std::ffi::CStr;

pub struct ElecSys {
	elec: *mut elec_t
}

impl ElecSys {
	pub fn new(filename: &str) -> Option<ElecSys> {
		let elec = unsafe {
			let c_filename = CString::new(filename).unwrap();
			libelec_new(c_filename.as_ptr())
		};
		if !elec.is_null() {
			Some(ElecSys{elec: elec})
		} else {
			None
		}
	}
	pub fn start(&mut self) -> bool {
		unsafe {
			libelec_sys_start(self.elec)
		}
	}
	pub fn stop(&mut self) {
		unsafe {
			libelec_sys_stop(self.elec)
		}
	}
	pub fn is_started(&self) -> bool {
		unsafe {
			libelec_sys_is_started(self.elec)
		}
	}
	pub fn can_start(&self) -> bool {
		unsafe {
			libelec_sys_can_start(self.elec)
		}
	}
	pub fn sys_set_time_factor(&mut self, time_factor: f64) {
		unsafe {
			libelec_sys_set_time_factor(self.elec, time_factor)
		}
	}
	pub fn sys_get_time_factor(&self) -> f64 {
		unsafe {
			libelec_sys_get_time_factor(self.elec)
		}
	}
	pub fn add_user_cb(&mut self, pre: bool, cb: elec_user_cb_t,
	    userinfo: *mut c_void) {
		unsafe {
			libelec_add_user_cb(self.elec, pre, cb, userinfo)
		}
	}
	pub fn remove_user_cb(&mut self, pre: bool, cb: elec_user_cb_t,
	    userinfo: *mut c_void) {
		unsafe {
			libelec_remove_user_cb(self.elec, pre, cb, userinfo)
		}
	}
	pub fn comp_find(&self, name: &str) -> Option<ElecComp> {
		let comp = unsafe {
			let c_name = CString::new(name).unwrap();
			libelec_comp_find(self.elec, c_name.as_ptr())
		};
		if !comp.is_null() {
			Some(ElecComp{comp: comp})
		} else {
			None
		}
	}
	extern "C" fn comp_walk_cb(comp: *mut elec_comp_t,
	    userinfo: *mut c_void) {
		unsafe {
			let comps = userinfo as *mut Vec<ElecComp>;
			(*comps).push(ElecComp{comp: comp})
		}
	}
	pub fn all_comps(&self) -> Vec<ElecComp> {
		let mut comps: Vec<ElecComp> = vec![];
		unsafe {
			let comps_ptr: *mut Vec<ElecComp> = &mut comps;
			libelec_walk_comps(self.elec, Self::comp_walk_cb,
			    comps_ptr as *mut c_void)
		};
		comps
	}
}

impl Drop for ElecSys {
	fn drop(&mut self) {
		unsafe {
			if libelec_sys_is_started(self.elec) {
				libelec_sys_stop(self.elec);
			}
			libelec_destroy(self.elec);
		}
	}
}

#[derive(Clone, Copy)]
pub struct ElecComp {
	comp: *mut elec_comp_t
}

#[derive(Debug, PartialEq)]
#[repr(C)]
pub enum CompType {
	Batt,
	Gen,
	TRU,
	Inv,
	Load,
	Bus,
	CB,
	Shunt,
	Tie,
	Diode,
	LabelBox
}

impl ElecComp {
	/*
	 * Configuration interrogation
	 */
	pub fn get_name(&self) -> String {
		unsafe {
			CStr::from_ptr(libelec_comp_get_name(self.comp))
			    .to_str()
			    .unwrap()
			    .to_string()
		}
	}
	fn get_type(&self) -> CompType {
		unsafe {
			libelec_comp_get_type(self.comp)
		}
	}
	pub fn get_location(&self) -> String {
		unsafe {
			CStr::from_ptr(libelec_comp_get_location(self.comp))
			    .to_str()
			    .unwrap()
			    .to_string()
		}
	}
	pub fn get_autogen(&self) -> bool {
		unsafe { libelec_comp_get_autogen(self.comp) }
	}
	pub fn get_num_conns(&self) -> usize {
		unsafe { libelec_comp_get_num_conns(self.comp) }
	}
	pub fn get_conn(&self, i: usize) -> ElecComp {
		unsafe {
			assert!(i < Self::get_num_conns(self));
			ElecComp{comp: libelec_comp_get_conn(self.comp, i)}
		}
	}
	#[allow(non_snake_case)]
	pub fn is_AC(&self) -> bool {
		unsafe { libelec_comp_is_AC(self.comp) }
	}
	/*
	 * Electrical state interrogation
	 */
	pub fn in_volts(&self) -> f64 {
		unsafe { libelec_comp_get_in_volts(self.comp) }
	}
	pub fn out_volts(&self) -> f64 {
		unsafe { libelec_comp_get_out_volts(self.comp) }
	}
	pub fn in_amps(&self) -> f64 {
		unsafe { libelec_comp_get_in_amps(self.comp) }
	}
	pub fn out_amps(&self) -> f64 {
		unsafe { libelec_comp_get_out_amps(self.comp) }
	}
	pub fn in_pwr(&self) -> f64 {
		unsafe { libelec_comp_get_in_pwr(self.comp) }
	}
	pub fn out_pwr(&self) -> f64 {
		unsafe { libelec_comp_get_out_pwr(self.comp) }
	}
	pub fn in_freq(&self) -> f64 {
		unsafe { libelec_comp_get_in_freq(self.comp) }
	}
	pub fn out_freq(&self) -> f64 {
		unsafe { libelec_comp_get_out_freq(self.comp) }
	}
	pub fn incap_volts(&self) -> f64 {
		unsafe { libelec_comp_get_incap_volts(self.comp) }
	}
	pub fn is_powered(&self) -> bool {
		unsafe { libelec_comp_is_powered(self.comp) }
	}
	pub fn get_eff(&self) -> f64 {
		unsafe { libelec_comp_get_eff(self.comp) }
	}
	pub fn get_srcs(&self) -> Vec<ElecComp> {
		let mut srcs_array: [*mut elec_comp_t; ELEC_MAX_SRCS] =
		    [std::ptr::null_mut(); ELEC_MAX_SRCS];
		let n = unsafe {
			libelec_comp_get_srcs(self.comp, &mut srcs_array)
		};
		let mut srcs: Vec<ElecComp> = vec![];
		for i in 0 .. n {
			srcs.push(ElecComp{ comp: srcs_array[i] });
		}
		srcs
	}
	/*
	 * Failures
	 */
	pub fn set_failed(&mut self, failed: bool) {
		unsafe { libelec_comp_set_failed(self.comp, failed) }
	}
	pub fn get_failed(&self) -> bool {
		unsafe { libelec_comp_get_failed(self.comp) }
	}
	pub fn set_shorted(&mut self, shorted: bool) {
		unsafe { libelec_comp_set_shorted(self.comp, shorted) }
	}
	pub fn get_shorted(&self) -> bool {
		unsafe { libelec_comp_get_shorted(self.comp) }
	}
	pub fn set_random_volts(&mut self, stddev: f64) -> f64 {
		unsafe { libelec_gen_set_random_volts(self.comp, stddev) }
	}
	pub fn set_random_freq(&mut self, stddev: f64) -> f64 {
		unsafe { libelec_gen_set_random_freq(self.comp, stddev) }
	}
	/*
	 * CBs
	 */
	pub fn cb_set(&mut self, set: bool) {
		assert_eq!(self.get_type(), CompType::CB);
		unsafe { libelec_cb_set(self.comp, set) }
	}
	pub fn cb_get(&self) -> bool {
		assert_eq!(self.get_type(), CompType::CB);
		unsafe { libelec_cb_get(self.comp) }
	}
	pub fn cb_get_temp(&self) -> f64 {
		assert_eq!(self.get_type(), CompType::CB);
		unsafe { libelec_cb_get_temp(self.comp) }
	}
	/*
	 * Ties
	 */
	pub fn tie_set_list(&mut self, list: &Vec<ElecComp>) {
		assert_eq!(self.get_type(), CompType::Tie);
		let comps: Vec<*const elec_comp_t> = list.iter()
		    .map(|c| c.comp as *const elec_comp_t)
		    .collect();
		unsafe {
			libelec_tie_set_list(self.comp, comps.len(),
			    comps.as_ptr() as *const*const elec_comp_t)
		}
	}
	pub fn tie_set_all(&mut self, tied: bool) {
		assert_eq!(self.get_type(), CompType::Tie);
		unsafe { libelec_tie_set_all(self.comp, tied) }
	}
	pub fn tie_get_all(&self) -> bool {
		assert_eq!(self.get_type(), CompType::Tie);
		unsafe { libelec_tie_get_all(self.comp) }
	}
	pub fn tie_get_list(&self) -> Vec<ElecComp> {
		assert_eq!(self.get_type(), CompType::Tie);
		let n_comps = unsafe { libelec_tie_get_num_buses(self.comp) };
		let mut comps: Vec<*mut elec_comp_t> =
		    vec![std::ptr::null_mut(); n_comps];
		unsafe {
			libelec_tie_get_list(self.comp, n_comps,
			    comps.as_mut_ptr() as *mut*mut elec_comp_t);
		};
		comps.into_iter()
		    .map(|c| ElecComp{comp: c})
		    .collect()
	}
	pub fn tie_get_num_buses(&self) -> usize {
		assert_eq!(self.get_type(), CompType::Tie);
		unsafe { libelec_tie_get_num_buses(self.comp) }
	}
	/*
	 * Batteries
	 */
	pub fn batt_get_chg_rel(&self) -> f64 {
		assert_eq!(self.get_type(), CompType::Batt);
		unsafe { libelec_batt_get_chg_rel(self.comp) }
	}
	pub fn batt_set_chg_rel(&mut self, chg_rel: f64) {
		assert_eq!(self.get_type(), CompType::Batt);
		unsafe { libelec_batt_set_chg_rel(self.comp, chg_rel) }
	}
	pub fn batt_get_temp(&self) -> f64 {
		assert_eq!(self.get_type(), CompType::Batt);
		unsafe { libelec_batt_get_temp(self.comp) }
	}
	#[allow(non_snake_case)]
	pub fn batt_set_temp(&mut self, T: f64) {
		assert_eq!(self.get_type(), CompType::Batt);
		unsafe { libelec_batt_set_temp(self.comp, T) }
	}
	/*
	 * Chargers
	 */
	pub fn chgr_get_working(&self) -> bool {
		assert_eq!(self.get_type(), CompType::TRU);
		unsafe { libelec_chgr_get_working(self.comp) }
	}
}

/*
 * libelec C interface
 */
type elec_user_cb_t = extern "C" fn(*mut c_void);
type elec_comp_walk_cb_t = extern "C" fn(*mut elec_comp_t, *mut c_void);
type elec_get_temp_cb_t = extern "C" fn(*mut elec_comp_t,
    userinfo: *mut c_void) -> f64;
type elec_get_rpm_cb_t = extern "C" fn(*mut elec_comp_t,
    userinfo: *mut c_void) -> f64;
type elec_get_load_cb_t = extern "C" fn(*mut elec_comp_t,
    userinfo: *mut c_void) -> f64;

#[repr(C)]
pub struct elec_t {
	_unused: [u8; 0],
}

#[repr(C)]
pub struct elec_comp_t {
	_unused: [u8; 0],
}

const ELEC_MAX_SRCS: usize =	64;

extern "C" {
	fn libelec_new(filename: *const c_char) -> *mut elec_t;
	fn libelec_destroy(elec: *mut elec_t);

	fn libelec_sys_start(elec: *mut elec_t) -> bool;
	fn libelec_sys_stop(elec: *mut elec_t);
	fn libelec_sys_is_started(elec: *const elec_t) -> bool;
	fn libelec_sys_can_start(elec: *const elec_t) -> bool;

	fn libelec_sys_set_time_factor(elec: *mut elec_t, time_factor: f64);
	fn libelec_sys_get_time_factor(elec: *const elec_t) -> f64;

	fn libelec_add_user_cb(elec: *mut elec_t, pre: bool,
	    cb: elec_user_cb_t, userinfo: *mut c_void);
	fn libelec_remove_user_cb(elec: *mut elec_t, pre: bool,
	    cb: elec_user_cb_t, userinfo: *mut c_void);

	fn libelec_comp_find(elec: *const elec_t, name: *const c_char) ->
	    *mut elec_comp_t;
	fn libelec_walk_comps(elec: *const elec_t, cb: elec_comp_walk_cb_t,
	    userinfo: *mut c_void);

	fn libelec_comp_get_num_conns(comp: *const elec_comp_t) -> usize;
	fn libelec_comp_get_conn(comp: *const elec_comp_t, i: usize) ->
	    *mut elec_comp_t;

	fn libelec_comp_is_AC(comp: *const elec_comp_t) -> bool;
	fn libelec_comp_get_name(comp: *const elec_comp_t) -> *const c_char;
	fn libelec_comp_get_type(comp: *const elec_comp_t) -> CompType;
	fn libelec_comp_get_location(comp: *const elec_comp_t) -> *const c_char;
	fn libelec_comp_get_autogen(comp: *const elec_comp_t) -> bool;

	fn libelec_comp_get_in_volts(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_out_volts(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_in_amps(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_out_amps(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_in_pwr(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_out_pwr(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_in_freq(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_out_freq(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_incap_volts(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_is_powered(comp: *const elec_comp_t) -> bool;
	fn libelec_comp_get_eff(comp: *const elec_comp_t) -> f64;
	fn libelec_comp_get_srcs(comp: *const elec_comp_t,
	    srcs: &mut [*mut elec_comp_t; ELEC_MAX_SRCS]) -> usize;

	fn libelec_comp_set_failed(comp: *mut elec_comp_t, failed: bool);
	fn libelec_comp_get_failed(comp: *const elec_comp_t) -> bool;
	fn libelec_comp_set_shorted(comp: *mut elec_comp_t, shorted: bool);
	fn libelec_comp_get_shorted(comp: *const elec_comp_t) -> bool;
	fn libelec_gen_set_random_volts(comp: *mut elec_comp_t,
	    stddev: f64) -> f64;
	fn libelec_gen_set_random_freq(comp: *mut elec_comp_t,
	    stddev: f64) -> f64;

	fn libelec_comp_set_userinfo(comp: *mut elec_comp_t,
	    userinfo: *mut c_void);
	fn libelec_comp_get_userinfo(comp: *const elec_comp_t) ->
	    *mut c_void;

	fn libelec_batt_set_temp_cb(batt: *mut elec_comp_t,
	    cb: elec_get_temp_cb_t);
	fn libelec_batt_get_temp_cb(batt: *const elec_comp_t) ->
	    elec_get_temp_cb_t;

	fn libelec_gen_set_rpm_cb(gen: *mut elec_comp_t,
	    cb: elec_get_rpm_cb_t);
	fn libelec_gen_get_rpm_cb(gen: *const elec_comp_t) ->
	    elec_get_rpm_cb_t;

	fn libelec_load_set_load_cb(load: *mut elec_comp_t,
	    cb: elec_get_load_cb_t);
	fn libelec_load_get_load_cb(load: *const elec_comp_t) ->
	    elec_get_load_cb_t;

	fn libelec_cb_set(comp: *mut elec_comp_t, set: bool);
	fn libelec_cb_get(comp: *const elec_comp_t) -> bool;
	fn libelec_cb_get_temp(comp: *const elec_comp_t) -> f64;

	fn libelec_tie_set_list(comp: *mut elec_comp_t, list_len: usize,
	    bus_list: *const*const elec_comp_t);
	fn libelec_tie_set_all(comp: *mut elec_comp_t, tied: bool);
	fn libelec_tie_get_all(comp: *const elec_comp_t) -> bool;
	fn libelec_tie_get_list(comp: *const elec_comp_t, cap: usize,
	    bus_list: *mut*mut elec_comp_t) -> usize;
	fn libelec_tie_get_num_buses(comp: *const elec_comp_t) -> usize;

	fn libelec_batt_get_chg_rel(batt: *const elec_comp_t) -> f64;
	fn libelec_batt_set_chg_rel(batt: *mut elec_comp_t, chg_rel: f64);
	fn libelec_batt_get_temp(batt: *const elec_comp_t) -> f64;
	fn libelec_batt_set_temp(batt: *mut elec_comp_t, T: f64);

	fn libelec_chgr_get_working(chgr: *const elec_comp_t) -> bool;

	pub fn libelec_phys_get_batt_voltage(U_nominal: f64, chg_rel: f64,
	    I_rel: f64) -> f64;
}

mod tests {
	const TEST_NET_FILE: &str = "../nettest/test.net";
	extern "C" {
		fn crc64_init();
	}
	#[test]
	fn load_net() {
		use crate::ElecSys;
		unsafe { crc64_init() };
		ElecSys::new(TEST_NET_FILE)
		    .expect(&format!("Failed to load net {}", TEST_NET_FILE));
	}
	#[test]
	fn load_and_run_net() {
		use crate::ElecSys;
		unsafe { crc64_init() };
		let mut sys = ElecSys::new(TEST_NET_FILE)
		    .expect(&format!("Failed to load net {}", TEST_NET_FILE));
		sys.start();
		std::thread::sleep(std::time::Duration::new(1, 0));
		for comp in sys.all_comps().iter() {
			if !comp.get_autogen() {
				println!(concat!(
				    "{} ({:?}) U_in:{:.1}V  U_out:{:.1}V  ",
				    "I_in:{:.1}A  I_out:{:.1}A  ",
				    "W_in:{:.1}W  W_out:{:.1}W"),
				    comp.get_name(), comp.get_type(),
				    comp.in_volts(), comp.out_volts(),
				    comp.in_amps(), comp.out_amps(),
				    comp.in_pwr(), comp.out_pwr());
			}
		}
	}
	#[test]
	fn list_all_comps() {
		use crate::ElecSys;
		unsafe { crc64_init() };
		let sys = ElecSys::new(TEST_NET_FILE)
		    .expect(&format!("Failed to load net {}", TEST_NET_FILE));
		for comp in sys.all_comps().iter() {
			if !comp.get_autogen() {
				println!(concat!(
				    "{} of type {:?}; location: \"{}\"; ",
				    "U_in:{}V  U_out:{}V  I_in:{}A  ",
				    "I_out:{}A  W_in:{}W  W_out:{}W"),
				    comp.get_name(), comp.get_type(),
				    comp.get_location(),
				    comp.in_volts(), comp.out_volts(),
				    comp.in_amps(), comp.out_amps(),
				    comp.in_pwr(), comp.out_pwr());
			}
		}
	}
}
