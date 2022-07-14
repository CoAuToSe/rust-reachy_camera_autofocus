#[path = "side_zoom_focus.rs"]
pub mod side_zoom_focus;
pub use side_zoom_focus::*;

#[macro_export]
macro_rules! st {
    ($a:expr) => {
        String::from($a)
    };
}
#[macro_export]
macro_rules! stvec {
    ($($a:expr),*) => (
        vec![$(st!($a)),*]
    )
}
#[macro_export]
macro_rules! am {
    ($e:expr) => {
        Arc::new(Mutex::new($e))
    };
}
#[macro_export]
macro_rules! amvec {
    ($($a:expr),*) => (
        vec![$(am!($a)),*]
    )
}
#[macro_export]
macro_rules! printol {
    ($tts:tt) => {
        print!("\r");
        print!($tts);
        std::io::stdout().flush().unwrap();
    };
}
#[macro_export]
#[allow(unused_mut)]
macro_rules! arc_mutex {
    ($name_arc:ident => $name_var:ident) => {
        #[cfg(not(debug_assertions))]
        {
            $name_var = $name_arc.lock().unwrap();
        }
        #[cfg(debug_assertions)]
        loop {
            if let Ok(some) = $name_arc.try_lock() {
                $name_var = some;
                println!(
                    "macro getting lock of {}: {:?} at {:?}",
                    stringify!($name_arc),
                    line!(),
                    std::time::Instant::now()
                );
                break;
            } else {
                std::thread::sleep(std::time::Duration::from_millis(100));
                println!("macro {}: {:?}", stringify!($name_var), line!());
            }
        }
    };
}
#[macro_export]
macro_rules! ameis {
    ($am_var:ident => $var:ident; $some:block ) => {
        {
            #[allow(unused_mut)]
            let mut $var;
            arc_mutex!($am_var=>$var);
            $some
        }
    };
}

#[inline]
pub fn inside<T: std::cmp::PartialEq>(to_compate: &T, rtgze: Vec<T>) -> bool {
    for e in rtgze {
        if e == *to_compate {
            return true;
        }
    }
    return false;
}
#[doc(alias("index"))]
pub fn eye_to_index(eye: &String) -> usize {
    match eye as &str {
        "left_eye" | "left" => 0,
        "right_eye" | "right" => 1,
        _ => panic!("can't recognize"),
    }
}
#[doc(alias("index"))]
#[inline]
pub fn side_to_index(side: SideOnly) -> usize {
    match side {
        SideOnly::Left => 0,
        SideOnly::Right => 1,
    }
}
#[inline]
pub fn min<T: std::cmp::PartialOrd>(a: T, b: T) -> T {
    if a > b {
        b
    } else {
        a
    }
}
#[inline]
pub fn max<T: std::cmp::PartialOrd>(a: T, b: T) -> T {
    if a < b {
        b
    } else {
        a
    }
}
