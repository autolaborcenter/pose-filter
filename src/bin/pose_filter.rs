use nalgebra::{Isometry2, Vector2};

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
                            Some(q) => { &isometry * q }
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

/// 从字符串解析等距映射
fn parse_isometry2(s: &str) -> Option<Isometry2<f64>> {
    let mut i = 0;
    let mut numbers = [0.0; 3];
    for r in s.split(',').map(|s| s.trim().parse::<f64>()) {
        if i >= numbers.len() || r.is_err() { return None; }
        numbers[i] = r.unwrap();
        i += 1;
    }
    Some(Isometry2::new(Vector2::new(numbers[0], numbers[1]), numbers[2]))
}

#[test]
fn parse_test() {
    assert_eq!(parse_isometry2("-1,+2,-0"), Some(Isometry2::new(Vector2::new(-1.0, 2.0), 0.0)));
    assert_eq!(parse_isometry2("1,2,3,x"), None);
    assert_eq!(parse_isometry2(""), None);
}
