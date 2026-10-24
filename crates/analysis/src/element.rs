//! The trilinear hexahedral element every voxel becomes.

use crate::material::{MaterialError, OrthotropicMaterial};

/// Natural-coordinate corner signs of the reference hex. The order fixes the node numbering of
/// every element: four corners of the bottom face counter-clockwise, then the top face.
pub(crate) const CORNER: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// One element's stiffness and constitutive matrices, both divided by the material's reference
/// modulus so the solver carries a single scalar E and multiplies by it.
#[derive(Debug, Clone)]
pub struct Element {
    /// Physical stiffness / E for element size `h`: `(h/2) Σ_gauss Bᵀ C B`.
    pub ke: Box<[[f64; 24]; 24]>,
    /// Constitutive matrix / E in Voigt order (xx, yy, zz, xy, yz, zx).
    pub c: [[f64; 6]; 6],
}

impl Element {
    /// Builds the element for a material and a voxel size `h` (metres).
    pub fn build(material: &OrthotropicMaterial, h: f64) -> Result<Element, MaterialError> {
        material.validate()?;
        let reference = material.reference();
        let ex = material.ex / reference;
        let ey = material.ey / reference;
        let ez = material.ez / reference;
        let (nu_xy, nu_yz, nu_zx) = (material.nu_xy, material.nu_yz, material.nu_zx);

        // Compliance, then invert. The 3×3 normal block is coupled; the shear terms are not.
        // Symmetry of the compliance matrix fixes the minor ratios: nu_yx/ey = nu_xy/ex, and so on.
        let s = [[1.0 / ex, -nu_xy / ex, -nu_zx / ez], [-nu_xy / ex, 1.0 / ey, -nu_yz / ey], [-nu_zx / ez, -nu_yz / ey, 1.0 / ez]];
        let determinant = s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1]) - s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0])
            + s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
        if !(determinant.abs() > 1e-30) || !determinant.is_finite() {
            return Err(MaterialError::Inconsistent);
        }
        let cofactor = |row: usize, column: usize| {
            let (r1, r2) = ((row + 1) % 3, (row + 2) % 3);
            let (c1, c2) = ((column + 1) % 3, (column + 2) % 3);
            s[r1][c1] * s[r2][c2] - s[r1][c2] * s[r2][c1]
        };
        let mut c = [[0.0; 6]; 6];
        for (i, row) in c.iter_mut().take(3).enumerate() {
            for (j, value) in row.iter_mut().take(3).enumerate() {
                *value = cofactor(j, i) / determinant; // inverse = adjugate / det
            }
        }
        c[3][3] = material.gxy / reference;
        c[4][4] = material.gyz / reference;
        c[5][5] = material.gzx / reference;

        // Turn the material's own frame onto the direction its fibre runs in. Done as the
        // fourth-order tensor rotation C'_ijkl = R_ip R_jq R_kr R_ls C_pqrs rather than through a
        // Voigt (Bond) matrix: the Voigt route has different transforms for stress and engineering
        // strain, and a wrong factor of two there is silent. Eighty-one terms once per solve is
        // cheap.
        if let Some(axis) = material.fibre_direction.normalized() {
            let ax = [axis.x, axis.y, axis.z];
            let helper = if ax[2].abs() > 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 0.0, 1.0] };
            let mut ay = [ax[1] * helper[2] - ax[2] * helper[1], ax[2] * helper[0] - ax[0] * helper[2], ax[0] * helper[1] - ax[1] * helper[0]];
            let y_len = (ay[0] * ay[0] + ay[1] * ay[1] + ay[2] * ay[2]).sqrt();
            if y_len > 0.0 {
                for v in &mut ay {
                    *v /= y_len;
                }
            }
            let az = [ax[1] * ay[2] - ax[2] * ay[1], ax[2] * ay[0] - ax[0] * ay[2], ax[0] * ay[1] - ax[1] * ay[0]];
            // R maps a vector written in the material frame into the model: its columns are the
            // material axes seen from the model.
            let r = [[ax[0], ay[0], az[0]], [ax[1], ay[1], az[1]], [ax[2], ay[2], az[2]]];
            const VOIGT: [[usize; 2]; 6] = [[0, 0], [1, 1], [2, 2], [0, 1], [1, 2], [2, 0]];
            let mut tensor = [[[[0.0_f64; 3]; 3]; 3]; 3];
            for a in 0..6 {
                for b in 0..6 {
                    let [i, j] = VOIGT[a];
                    let [k, l] = VOIGT[b];
                    let value = c[a][b];
                    tensor[i][j][k][l] = value;
                    tensor[j][i][k][l] = value;
                    tensor[i][j][l][k] = value;
                    tensor[j][i][l][k] = value;
                }
            }
            let mut rotated = [[[[0.0_f64; 3]; 3]; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    for k in 0..3 {
                        for l in 0..3 {
                            let mut sum = 0.0;
                            for p in 0..3 {
                                for q in 0..3 {
                                    for rr in 0..3 {
                                        for ss in 0..3 {
                                            sum += r[i][p] * r[j][q] * r[k][rr] * r[l][ss] * tensor[p][q][rr][ss];
                                        }
                                    }
                                }
                            }
                            rotated[i][j][k][l] = sum;
                        }
                    }
                }
            }
            for a in 0..6 {
                for b in 0..6 {
                    c[a][b] = rotated[VOIGT[a][0]][VOIGT[a][1]][VOIGT[b][0]][VOIGT[b][1]];
                }
            }
        }

        let mut ke = Box::new([[0.0_f64; 24]; 24]);
        let g = 1.0 / 3.0_f64.sqrt();
        for xi in [-g, g] {
            for eta in [-g, g] {
                for zeta in [-g, g] {
                    let mut b = [[0.0_f64; 24]; 6];
                    for (n, corner) in CORNER.iter().enumerate() {
                        let [sx, sy, sz] = *corner;
                        let dx = 0.125 * sx * (1.0 + sy * eta) * (1.0 + sz * zeta);
                        let dy = 0.125 * sy * (1.0 + sx * xi) * (1.0 + sz * zeta);
                        let dz = 0.125 * sz * (1.0 + sx * xi) * (1.0 + sy * eta);
                        b[0][3 * n] = dx;
                        b[1][3 * n + 1] = dy;
                        b[2][3 * n + 2] = dz;
                        b[3][3 * n] = dy;
                        b[3][3 * n + 1] = dx;
                        b[4][3 * n + 1] = dz;
                        b[4][3 * n + 2] = dy;
                        b[5][3 * n] = dz;
                        b[5][3 * n + 2] = dx;
                    }
                    // Bᵀ C B, with C B formed once per column.
                    let mut cb = [[0.0_f64; 24]; 6];
                    for p in 0..6 {
                        for col in 0..24 {
                            let mut sum = 0.0;
                            for q in 0..6 {
                                sum += c[p][q] * b[q][col];
                            }
                            cb[p][col] = sum;
                        }
                    }
                    for a in 0..24 {
                        for bb in 0..24 {
                            let mut sum = 0.0;
                            for p in 0..6 {
                                sum += b[p][a] * cb[p][bb];
                            }
                            ke[a][bb] += sum;
                        }
                    }
                }
            }
        }
        for row in ke.iter_mut() {
            for value in row.iter_mut() {
                *value *= h / 2.0;
            }
        }
        Ok(Element { ke, c })
    }
}
