use nalgebra::{Isometry2, Vector2};
use rtk_ins570_rs::ins570::{Solution, SolutionState};
use rtk_ins570_rs::*;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

fn main() {
    let pose_rtk = Arc::new(Mutex::new((
        Instant::now(),
        Isometry2::new(Vector2::default(), 0.0),
    )));
    let info = Arc::new(Mutex::new(String::from("nothing")));

    let info_copy = info.clone();
    let pose_copy = pose_rtk.clone();
    std::thread::spawn(move || loop {
        let pose = pose_copy.clone();
        let info = info_copy.clone();
        rtk_threads!(move |_, rtk| {
            for isometry in rtk.filter_map(|solution| match solution {
                Solution::Uninitialized(SolutionState {
                    state_pos,
                    state_dir,
                    satellites,
                }) => {
                    *info.lock().unwrap() =
                        format!("uninitialized: {} {} {}", state_pos, state_dir, satellites);
                    None
                }
                Solution::Data(data) => {
                    let SolutionState {
                        state_pos,
                        state_dir,
                        satellites,
                    } = data.state;
                    if state_pos < 40 || state_dir < 40 {
                        *info.lock().unwrap() =
                            format!("unlocked: {} {} {}", state_pos, state_dir, satellites);
                        None
                    } else {
                        info.lock().unwrap().clear();
                        Some(Isometry2::new(
                            Vector2::new(data.enu.e, data.enu.n),
                            data.dir,
                        ))
                    }
                }
            }) {
                *pose.lock().unwrap() = (Instant::now(), isometry);
            }
        })
        .join();
        std::thread::sleep(Duration::from_secs(1));
    });

    let mut isometry = Isometry2::new(Vector2::new(0.0, 0.0), 0.0);
    let mut line = String::new();

    loop {
        line.clear();
        match std::io::stdin().read_line(&mut line) {
            Ok(_) => {
                let now = Instant::now();
                let robot = match parse_isometry2(line.trim()) {
                    Some(i) => i, // 变换到上一次匹配时的机器人系
                    None => continue,
                };
                let (stamp, rtk) = *pose_rtk.lock().unwrap();
                if now.duration_since(stamp) < Duration::from_millis(100) {
                    isometry = rtk * robot.inverse();
                }

                let a = &isometry * robot;
                println!(
                    "A {},{},{} {}",
                    a.translation.x,
                    a.translation.y,
                    a.rotation.angle(),
                    info.lock().unwrap().as_str(),
                )
            }
            Err(_) => break,
        }
    }
}

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<Isometry2<f64>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f64>()) {
        if i >= numbers.len() || r.is_err() {
            return None;
        }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(Isometry2::new(
        Vector2::new(numbers[0], numbers[1]),
        numbers[2],
    ))
}

#[test]
fn parse_test() {
    assert_eq!(
        parse_isometry2("-1,+2,-0"),
        Some(Isometry2::new(Vector2::new(-1.0, 2.0), 0.0))
    );
    assert_eq!(parse_isometry2("1,2,3,x"), None);
    assert_eq!(parse_isometry2(""), None);
}
