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

/// Camera axes (x right, y down, z forward, as the video shows them) in device axes (x right, y up, z out
/// of the screen in portrait), for `screen_deg` = `screen.orientation.angle`: the image turns with the screen.
// shortcut: sign of the landscape turn follows the W3C definition, not yet checked on a phone.
pub fn camera_in_device(screen_deg: f32) -> M3 {
    mul(&rot_z(screen_deg.to_radians()), &[[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
}

/// `devicemotion.acceleration` (m/s^2, device axes) in camera axes. Negated: on the iPhone it correlates
/// with the visual acceleration at r -0.1..0 as given and 0.43-0.57 negated (two walk replays, 2026-11-26).
// shortcut: the sign is checked on iOS only; check an Android recording before shipping there.
pub fn accel_in_camera(a: V3, screen_deg: f32) -> V3 {
    scale(mulv(&transpose(&camera_in_device(screen_deg)), a), -1.0)
}

/// Camera-to-world rotation from W3C `deviceorientation` angles (degrees) and the screen angle.
/// Device frame to world is R = Rz(alpha) Rx(beta) Ry(gamma); the back camera flips y and z.
pub fn camera_from_orientation(alpha: f32, beta: f32, gamma: f32, screen_deg: f32) -> M3 {
    let r = mul(&mul(&rot_z(alpha.to_radians()), &rot_x(beta.to_radians())), &rot_y(gamma.to_radians()));
    mul(&r, &camera_in_device(screen_deg))
}

/// Midpoint of the closest approach of rays c1 + s d1 and c2 + t d2 (unit directions), if both
/// depths are positive and the rays nearly meet (gap under 2% of the nearer depth).
pub fn triangulate(c1: V3, d1: V3, c2: V3, d2: V3) -> Option<V3> {
    let w = sub(c1, c2);
    let b = dot(d1, d2);
    let den = 1.0 - b * b;
    if den < 1e-6 {
        return None; // parallel rays
    }
    let s = (b * dot(d2, w) - dot(d1, w)) / den;
    let t = dot(d2, w) + s * b;
    let (p1, p2) = (add(c1, scale(d1, s)), add(c2, scale(d2, t)));
    (s > 0.0 && t > 0.0 && norm(sub(p1, p2)) < 0.02 * s.min(t)).then(|| scale(add(p1, p2), 0.5))
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
        mulv(&transpose(&camera_from_orientation(alpha, beta, gamma, 0.0)), [0.0, 0.0, -1.0])
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
    fn triangulates_two_rays() {
        let x = [0.3, 2.0, 0.4];
        let (c1, c2) = ([0.0, 0.0, 1.0], [0.5, 0.0, 1.0]);
        let p = triangulate(c1, normalize(sub(x, c1)), c2, normalize(sub(x, c2))).unwrap();
        assert!(close(p, x), "{p:?}");
        assert!(triangulate(c1, [0.0, 1.0, 0.0], c2, [0.0, -1.0, 0.0]).is_none(), "behind");
    }

    #[test]
    fn solve_3x3() {
        let x = solve([[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]], [3.0, 5.0, 5.0]).unwrap();
        assert!(close(x, [1.0, 1.0, 1.0]));
        assert!(solve([[1.0, 2.0], [2.0, 4.0]], [1.0, 2.0]).is_none());
    }
}
