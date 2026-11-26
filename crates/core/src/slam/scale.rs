use super::*;

impl Slam {
    pub(super) fn estimate_scale(&mut self, accel: Option<V3>, dt: f32) {
        let Some(a) = accel else { return self.motion.clear() };
        let a = mulv(&self.r, a);
        let t = self.motion.last().map_or(0.0, |m| m.2) + dt;
        self.motion.push((self.c, a, t));
        while t - self.motion[0].2 > SCALE_WINDOW_S {
            self.motion.remove(0);
        }
        let n = self.motion.len();
        if t - self.motion[0].2 >= SCALE_WINDOW_S - 0.1 && self.frame % 30 == 0 {
            // Mean interval over the window: scale goes with dt^2, so one jittery frame must not set it.
            let dt = (t - self.motion[0].2) / (n - 1) as f32;
            let (p, imu): (Vec<V3>, Vec<V3>) = self.motion.iter().map(|m| (m.0, m.1)).unzip();
            if let Some((lag, s)) = crate::imu::scale_and_lag(&p, &imu, dt, ((MAX_LAG_S / dt).round() as usize).max(1)).filter(|e| SCALE_RANGE.contains(&e.1)) {
                (self.metres_per_unit, self.imu_lag_ms) = (Some(s), Some(lag as f32 * dt * 1e3));
            }
        }
    }
}
