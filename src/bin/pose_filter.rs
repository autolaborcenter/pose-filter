use std::ops::Mul;

use nalgebra::{Isometry2, Vector2};

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<Isometry2<f64>> {
    let numbers: Vec<_> = s
        .split(',')
        .map(|s| s.trim().parse::<f64>())
        .take_while(|r| r.is_ok())
        .map(|r| r.unwrap())
        .collect();
    if numbers.len() == 3 {
        Some(Isometry2::new(Vector2::new(numbers[0], numbers[1]), numbers[2]))
    } else {
        None
    }
}

fn main() {
    let mut isometry = Isometry2::new(Vector2::new(0.0, 0.0), 0.0);
    let mut line = String::new();

    loop {
        line.clear();
        match std::io::stdin().read_line(&mut line) {
            Ok(_) => {
                let words: Vec<_> = line.split(' ').collect();
                match words[0] {
                    // [P]air
                    "P" => if words.len() == 3 {
                        let rtk = match parse_isometry2(words[1]) {
                            Some(i) => { i } // 从上一次匹配的机器人系到 rtk 系
                            None => continue
                        };
                        let robot = match parse_isometry2(words[2]) {
                            Some(i) => { i.inverse() } // 变换到上一次匹配时的机器人系
                            None => continue
                        };
                        isometry = rtk * robot;
                        // println!("isometry = {}", isometry)
                    }
                    // [Q]uery -> [A]nswer
                    "Q" => if words.len() == 2 {
                        let a = match parse_isometry2(words[1]) {
                            Some(q) => { isometry.mul(q) }
                            None => continue
                        };
                        println!("A {},{},{}", a.translation.x, a.translation.y, a.rotation.angle())
                    }
                    // else
                    _ => {}
                }
            }
            Err(_) => break
        }
    }
}
