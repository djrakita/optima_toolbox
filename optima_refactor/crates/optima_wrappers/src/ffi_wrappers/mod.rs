use std::ffi::{c_int, CString};
use std::os::raw::{c_char, c_double};
use std::sync::OnceLock;
use optima_robotics::robot::ORobotDefault;
use std::sync::Mutex;
use once_cell::sync::Lazy;

pub mod ik_solvers;
pub mod ik_solvers2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


pub struct FFIConverters;
#[allow(dead_code)]
impl FFIConverters {
    pub (crate) unsafe fn c_str_to_rust_string(c_str: *const c_char) -> String {
        let c_str = std::ffi::CStr::from_ptr(c_str);
        let s = c_str.to_str().expect("Not a valid UTF-8 string");
        s.to_string()
    }

    pub (crate) unsafe fn rust_string_to_c_str(rust_string: String) -> *const c_char {
        let c_string = CString::new(rust_string).expect("error");
        c_string.into_raw() as *const c_char
    }

    pub (crate) unsafe fn c_int_to_rust_usize(i: c_int) -> usize {
        i as usize
    }

    pub (crate) unsafe fn rust_usize_to_c_int(u: usize) -> c_int {
        u as c_int
    }

    pub (crate) unsafe fn c_double_arr_to_rust_double_vec(c_float_arr: *const c_double, len: c_int) -> Vec<c_double> {
        let slice: &[c_double] = std::slice::from_raw_parts(c_float_arr, len as usize);
        slice.to_vec()
    }

    pub (crate) unsafe fn rust_f64_vec_to_c_double_arr(rust_float_vec: Vec<f64>) -> DoubleArray {
        let length = rust_float_vec.len() as c_int;
        let data = rust_float_vec.as_ptr() as *const c_double;
        std::mem::forget(rust_float_vec); // Prevent Rust from freeing the memory

        DoubleArray { data, length }
    }

    pub (crate) unsafe fn rust_vec_of_f64_vecs_to_array_of_double_arrays(rust_vec_of_f64_vecs: Vec<Vec<f64>>) -> ArrayOfDoubleArrays {
        let length = rust_vec_of_f64_vecs.len() as c_int;
        let mut double_arrays = Vec::with_capacity(length as usize);

        for f64_vec in rust_vec_of_f64_vecs {
            let inner_length = f64_vec.len() as c_int;
            let data = f64_vec.as_ptr() as *const c_double;
            std::mem::forget(f64_vec);

            double_arrays.push(DoubleArray { data, length: inner_length });
        }

        let data = double_arrays.as_ptr();
        std::mem::forget(double_arrays);

        ArrayOfDoubleArrays { data, length }
    }
}

#[repr(C)]
pub struct DoubleArray {
    pub data: *const c_double,
    pub length: c_int,
}

#[repr(C)]
pub struct ArrayOfDoubleArrays {
    pub data: *const DoubleArray,
    pub length: c_int
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pub static GLOBAL_ROBOT: Lazy<Mutex<Option<ORobotDefault>>> = Lazy::new(|| Mutex::new(None));

pub fn set_global_robot(robot_name: &str) {
    let mut robot_lock = GLOBAL_ROBOT.lock().unwrap();
    *robot_lock = Some(ORobotDefault::load_from_saved_robot(robot_name));
}

pub fn clear_global_robot() {
    let mut robot_lock = GLOBAL_ROBOT.lock().unwrap();
    *robot_lock = None;
}



#[no_mangle]
pub unsafe extern "C" fn ffi_set_global_robot(robot_name: *const c_char) {
    let s = FFIConverters::c_str_to_rust_string(robot_name);
    set_global_robot(&s);
}

#[no_mangle]
pub unsafe extern "C" fn ffi_clear_global_robot() {
    clear_global_robot();
}