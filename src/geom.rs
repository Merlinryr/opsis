//! Tiny 3D math: row-major 3x3 matrices, sensor orientation, small dense solve.
//! World frame = W3C DeviceOrientation earth frame: x east, y north, z up.
//! Camera frame = computer vision: x right, y down, z forward (out of the back camera).

pub type V3 = [f32; 3];
pub type M3 = [[f32; 3]; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, s: f32) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V3) -> f32 {
    dot(a, a).sqrt()
}
pub fn normalize(a: V3) -> V3 {
    scale(a, 1.0 / norm(a))
}

pub fn mulv(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}
pub fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}
pub fn mul(a: &M3, b: &M3) -> M3 {
    let bt = transpose(b);
    std::array::from_fn(|i| std::array::from_fn(|j| dot(a[i], bt[j])))
}

pub fn rot_x(t: f32) -> M3 {
    let (s, c) = t.sin_cos();
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}
pub fn rot_y(t: f32) -> M3 {
    let (s, c) = t.sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}
pub fn rot_z(t: f32) -> M3 {
    let (s, c) = t.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

/// Camera-to-world rotation from W3C `deviceorientation` angles (degrees), portrait screen.
/// Device frame (x right, y up, z out of the screen) is R = Rz(alpha) Rx(beta) Ry(gamma);
/// the back camera flips y and z.
// shortcut: portrait only; landscape needs a rotation by screen.orientation.angle.
pub fn camera_from_orientation(alpha: f32, beta: f32, gamma: f32) -> M3 {
    let r = mul(&mul(&rot_z(alpha.to_radians()), &rot_x(beta.to_radians())), &rot_y(gamma.to_radians()));
    mul(&r, &[[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
}

/// Solve the N x N system `a x = b` by Gaussian elimination with partial pivoting.
pub fn solve<const N: usize>(mut a: [[f32; N]; N], mut b: [f32; N]) -> Option<[f32; N]> {
    for c in 0..N {
        let p = (c..N).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-12 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..N {
            let k = a[r][c] / a[c][c];
            for j in c..N {
                a[r][j] -= k * a[c][j];
            }
            b[r] -= k * b[c];
        }
    }
    let mut x = [0.0; N];
    for r in (0..N).rev() {
        x[r] = (b[r] - (r + 1..N).map(|j| a[r][j] * x[j]).sum::<f32>()) / a[r][r];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: V3, b: V3) -> bool {
        norm(sub(a, b)) < 1e-5
    }

    /// Gravity (world -z) seen in camera axes.
    fn gravity_in_camera(alpha: f32, beta: f32, gamma: f32) -> V3 {
        mulv(&transpose(&camera_from_orientation(alpha, beta, gamma)), [0.0, 0.0, -1.0])
    }

    #[test]
    fn flat_screen_up_looks_down() {
        // W3C spec: alpha = beta = gamma = 0 is the device flat on a table, screen up.
        assert!(close(gravity_in_camera(0.0, 0.0, 0.0), [0.0, 0.0, 1.0]));
    }

    #[test]
    fn upright_portrait_looks_level() {
        // beta = 90: device upright, back camera level, gravity points down the image.
        assert!(close(gravity_in_camera(0.0, 90.0, 0.0), [0.0, 1.0, 0.0]));
        assert!(close(gravity_in_camera(123.0, 90.0, 0.0), [0.0, 1.0, 0.0]), "heading must not change gravity");
    }

    #[test]
    fn solve_3x3() {
        let x = solve([[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]], [3.0, 5.0, 5.0]).unwrap();
        assert!(close(x, [1.0, 1.0, 1.0]));
        assert!(solve([[1.0, 2.0], [2.0, 4.0]], [1.0, 2.0]).is_none());
    }
}
