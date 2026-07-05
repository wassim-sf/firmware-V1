//! Minimal `f32` linear algebra for SO(3) control — a hand-rolled stand-in for
//! the parts of `nalgebra` the simulator used, so the firmware pulls in no heavy
//! matrix crate. `Vec3`/`Mat3` plus the Lie-group helpers `hat`/`vee`/`so3_exp`
//! and a quaternion→rotation-matrix conversion. Ports `se3quad .../math.rs`.

use core::ops::{Add, Mul, Neg, Sub};

/// A 3-vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Unit vector along +x (the reference `Vector3::x()`).
    #[inline]
    pub const fn unit_x() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    #[inline]
    pub const fn unit_z() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    #[inline]
    pub fn dot(&self, o: &Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    #[inline]
    pub fn cross(&self, o: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    #[inline]
    pub fn norm_squared(&self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.norm_squared())
    }

    /// Unit vector; returns `self` unchanged if it is (near) zero.
    #[inline]
    pub fn normalize(&self) -> Vec3 {
        let n = self.norm();
        if n > 1e-12 {
            *self * (1.0 / n)
        } else {
            *self
        }
    }

    /// Apply `f` component-wise (mirrors `nalgebra`'s `map`, used for clamping).
    #[inline]
    pub fn map<F: Fn(f32) -> f32>(&self, f: F) -> Vec3 {
        Vec3::new(f(self.x), f(self.y), f(self.z))
    }

    #[inline]
    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    #[inline]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

/// A 3×3 matrix, stored row-major (`m[r][c]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    pub const ZERO: Mat3 = Mat3 {
        m: [[0.0; 3]; 3],
    };

    #[inline]
    pub const fn new(m: [[f32; 3]; 3]) -> Self {
        Self { m }
    }

    #[inline]
    pub fn identity() -> Self {
        Self::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    }

    /// Build a matrix from three **column** vectors (mirrors `from_columns`).
    #[inline]
    pub fn from_columns(c0: Vec3, c1: Vec3, c2: Vec3) -> Self {
        Self::new([
            [c0.x, c1.x, c2.x],
            [c0.y, c1.y, c2.y],
            [c0.z, c1.z, c2.z],
        ])
    }

    /// Diagonal matrix `diag(d)`.
    #[inline]
    pub fn from_diagonal(d: Vec3) -> Self {
        Self::new([[d.x, 0.0, 0.0], [0.0, d.y, 0.0], [0.0, 0.0, d.z]])
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f32 {
        self.m[r][c]
    }

    #[inline]
    pub fn transpose(&self) -> Mat3 {
        let m = &self.m;
        Mat3::new([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }

    #[inline]
    pub fn trace(&self) -> f32 {
        self.m[0][0] + self.m[1][1] + self.m[2][2]
    }

    #[inline]
    pub fn col(&self, c: usize) -> Vec3 {
        Vec3::new(self.m[0][c], self.m[1][c], self.m[2][c])
    }

    #[inline]
    pub fn is_finite(&self) -> bool {
        self.m.iter().flatten().all(|x| x.is_finite())
    }
}

impl Add for Mat3 {
    type Output = Mat3;
    #[inline]
    fn add(self, o: Mat3) -> Mat3 {
        let mut r = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                r.m[i][j] = self.m[i][j] + o.m[i][j];
            }
        }
        r
    }
}

impl Sub for Mat3 {
    type Output = Mat3;
    #[inline]
    fn sub(self, o: Mat3) -> Mat3 {
        let mut r = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                r.m[i][j] = self.m[i][j] - o.m[i][j];
            }
        }
        r
    }
}

impl Mul<f32> for Mat3 {
    type Output = Mat3;
    #[inline]
    fn mul(self, s: f32) -> Mat3 {
        let mut r = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                r.m[i][j] = self.m[i][j] * s;
            }
        }
        r
    }
}

impl Mul<Mat3> for f32 {
    type Output = Mat3;
    #[inline]
    fn mul(self, m: Mat3) -> Mat3 {
        m * self
    }
}

/// Matrix · vector.
impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }
}

/// Matrix · matrix.
impl Mul<Mat3> for Mat3 {
    type Output = Mat3;
    #[inline]
    fn mul(self, o: Mat3) -> Mat3 {
        let mut r = Mat3::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += self.m[i][k] * o.m[k][j];
                }
                r.m[i][j] = s;
            }
        }
        r
    }
}

/// `hat`: R^3 → so(3). `hat(v) * w == v.cross(w)`. Ports `matlab/hat.m`.
#[inline]
pub fn hat(v: &Vec3) -> Mat3 {
    Mat3::new([
        [0.0, -v.z, v.y],
        [v.z, 0.0, -v.x],
        [-v.y, v.x, 0.0],
    ])
}

/// `vee`: inverse of [`hat`] — `[m(2,1); m(0,2); m(1,0)]`. Ports `matlab/vee.m`.
#[inline]
pub fn vee(m: &Mat3) -> Vec3 {
    Vec3::new(m.get(2, 1), m.get(0, 2), m.get(1, 0))
}

/// SO(3) exponential map via Rodrigues' formula: `exp(hat(w))`.
pub fn so3_exp(w: &Vec3) -> Mat3 {
    let theta = w.norm();
    if theta < 1e-9 {
        return Mat3::identity() + hat(w);
    }
    let k = *w * (1.0 / theta);
    let kx = hat(&k);
    Mat3::identity() + (libm::sinf(theta) * kx) + ((1.0 - libm::cosf(theta)) * (kx * kx))
}

/// Rotation matrix (body→world) from a unit quaternion `q = [w, x, y, z]`.
///
/// Same convention as the firmware AHRS ([`crate::frame`] handles the Z-up→Z-down
/// world flip separately). Does not assume `q` is normalised — it normalises.
pub fn quat_to_mat(q: [f32; 4]) -> Mat3 {
    let n = libm::sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    let (w, x, y, z) = if n > 1e-12 {
        (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    } else {
        (1.0, 0.0, 0.0, 0.0)
    };
    Mat3::new([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    fn approx(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    fn mat_approx(a: &Mat3, b: &Mat3, tol: f32) -> bool {
        (0..3).all(|i| (0..3).all(|j| approx(a.get(i, j), b.get(i, j), tol)))
    }

    #[test]
    fn hat_vee_roundtrip() {
        let v = Vec3::new(0.3, -1.2, 0.7);
        assert!((vee(&hat(&v)) - v).norm() < 1e-6);
    }

    #[test]
    fn hat_is_cross_product() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(-0.5, 0.4, 2.1);
        assert!((hat(&a) * b - a.cross(&b)).norm() < 1e-6);
    }

    #[test]
    fn so3_exp_rotates_about_z() {
        let r = so3_exp(&Vec3::new(0.0, 0.0, PI / 2.0));
        // x-axis maps to y-axis.
        let x = Vec3::new(1.0, 0.0, 0.0);
        assert!((r * x - Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-5);
        // Valid rotation: RᵀR = I.
        assert!(mat_approx(&(r.transpose() * r), &Mat3::identity(), 1e-5));
    }

    #[test]
    fn matmul_and_transpose() {
        let a = Mat3::new([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]]);
        let at = a.transpose();
        assert_eq!(at.get(0, 1), a.get(1, 0));
        // (A B)ᵀ = Bᵀ Aᵀ sanity via identity.
        let i = Mat3::identity();
        assert_eq!(a * i, a);
    }

    #[test]
    fn from_columns_places_columns() {
        let c0 = Vec3::new(1.0, 2.0, 3.0);
        let c1 = Vec3::new(4.0, 5.0, 6.0);
        let c2 = Vec3::new(7.0, 8.0, 9.0);
        let m = Mat3::from_columns(c0, c1, c2);
        assert_eq!(m.col(0), c0);
        assert_eq!(m.col(1), c1);
        assert_eq!(m.col(2), c2);
    }

    #[test]
    fn quat_identity_is_identity_matrix() {
        let r = quat_to_mat([1.0, 0.0, 0.0, 0.0]);
        assert!(mat_approx(&r, &Mat3::identity(), 1e-6));
    }

    #[test]
    fn quat_yaw_90_matches_so3_exp() {
        // q for +90° about z = [cos45, 0, 0, sin45].
        let c = libm::cosf(PI / 4.0);
        let s = libm::sinf(PI / 4.0);
        let r = quat_to_mat([c, 0.0, 0.0, s]);
        let r2 = so3_exp(&Vec3::new(0.0, 0.0, PI / 2.0));
        assert!(mat_approx(&r, &r2, 1e-5));
    }

    #[test]
    fn quat_to_mat_is_orthonormal() {
        let r = quat_to_mat([0.5, 0.5, 0.5, 0.5]);
        let rtr = r.transpose() * r;
        for i in 0..3 {
            for j in 0..3 {
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!(approx(rtr.get(i, j), expect, 1e-5), "RᵀR not identity");
            }
        }
    }
}
