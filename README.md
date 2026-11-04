# Plane Truss Analysis in MATLAB

Teaching resources for analysing two-dimensional trusses using the **direct stiffness method**. The material is designed for a guided university class of approximately **one hour**, progressing from a two-member example to a six-member structure.

The repository combines a PDF tutorial, a MATLAB Live Script for the first exercise, and starter and completed MATLAB scripts for both exercises.

## Learning objectives

By completing the exercises, students should be able to:

- Represent a truss using node coordinates and member connectivity.
- Assign horizontal and vertical degrees of freedom (DOFs).
- Calculate member lengths, direction cosines and element stiffness matrices.
- Assemble the global stiffness matrix and impose support conditions.
- Calculate joint displacements, support reactions and axial forces.
- Check equilibrium and interpret a magnified deformed-shape plot.

## Repository contents

| Resource | Purpose |
| --- | --- |
| PDF tutorial | Structural assumptions, equations and step-by-step coding instructions |
| `exercise1_solution_tutorial.m` | Formatted explanations in livelink format and executable code for the first exercise |
| `exercise1_starter.m` | Exercise 1 before completion, with selected expressions to fill in |
| `exercise1_solution.m` | Completed Exercise 1 solution |
| `exercise2_starter.m` | Exercise 2 before completion, with selected expressions to fill in |
| `exercise2_solution.m` | Completed Exercise 2 solution |
| Workflow diagram | Visual summary of the calculation sequence |

The PDF and Live Script filenames depend on the versions uploaded. Identify the explanatory Live Script by its title, **Exercise 1 — Plane truss analysis step by step**. If it is distributed as `exercise1_solution.m`, it also serves as the completed first solution.

## Requirements

- Base MATLAB; no additional toolboxes are required.
- MATLAB R2025a or newer to display the plain-text Live Script (`.m`) as formatted content.
- A PDF reader for the tutorial.

In MATLAB, open the explanatory file in the **Live Editor**. If it opens in the ordinary Editor, right-click its document tab and select **Open as Live Script**. You can save an `.mlx` copy from the Live Editor using **Save As**. An `.m` extension alone does not distinguish a conventional script from a plain-text Live Script.

## Algorithm Explanation
The code follows the architecture presented in the diagram
<img width="811" height="1403" alt="image" src="https://github.com/user-attachments/assets/a99185fa-a338-4a53-a624-3fb24ba45106" />


## Exercise 1 — Two-member truss

### Problem statement

Determine the support reactions and axial force in each member of a two-member truss subjected to a **2 kN downward load at node 2**. Also calculate the displacement of the loaded joint.
<img width="315" height="381" alt="image" src="https://github.com/user-attachments/assets/7e19b960-6e6e-4fd9-a32f-efae1820a9e9" />


### Geometry and connectivity

| Node | x (m) | y (m) | Support condition |
| --- | ---: | ---: | --- |
| 1 | 3 | 4 | Horizontal and vertical displacements restrained |
| 2 | 0 | 0 | Free |
| 3 | 3 | 0 | Horizontal and vertical displacements restrained |

| Member | Start node | End node | Length (m) |
| --- | ---: | ---: | ---: |
| 1 | 2 | 3 | 3 |
| 2 | 2 | 1 | 5 |

Use **E = 200 GPa** and **A = 0.0015 m²** for both members. These are adopted teaching values: the supplied first textbook example specifies only that EA is constant.

### MATLAB input

```matlab
nodes = [3 4; 0 0; 3 0];
members = [2 3; 2 1];
E = 200e9;
A = 0.0015;
fixedDOFs = [1 2 5 6];
F = zeros(6,1);
F(4) = -2000;
```

### Reference results

| Quantity | Result |
| --- | ---: |
| Node 2 horizontal displacement | +0.015000 mm |
| Node 2 vertical displacement | −0.063333 mm |
| Member 1 axial force | −1.500 kN — compression |
| Member 2 axial force | +2.500 kN — tension |
| Node 1 horizontal reaction | +1.500 kN |
| Node 1 vertical reaction | +2.000 kN |
| Node 3 horizontal reaction | −1.500 kN |
| Node 3 vertical reaction | 0 kN |

## Exercise 2 — Six-member truss

### Problem statement

Determine the **vertical displacement at joint 2** and the **axial force in member 5**. A **30 kN downward load** acts at joint 3. All members have **A = 0.0015 m²** and **E = 200 GPa**.
<img width="471" height="331" alt="image" src="https://github.com/user-attachments/assets/226cf6e1-a2d1-4c3a-8db7-28b66fbc707c" />


The structure has two horizontal bays of 2 m and a height of 2 m. **Only nodes 1 and 5 are restrained at the wall.** The gusset plates at joints 2 and 4 are connections, not supports.

### Geometry and connectivity

| Node | x (m) | y (m) | Support condition |
| --- | ---: | ---: | --- |
| 1 | 0 | 0 | Horizontal and vertical displacements restrained |
| 2 | 2 | 0 | Free |
| 3 | 4 | 0 | Free |
| 4 | 2 | 2 | Free |
| 5 | 0 | 2 | Horizontal and vertical displacements restrained |

| Member | Start node | End node | Length (m) |
| --- | ---: | ---: | --- |
| 1 | 1 | 2 | 2 |
| 2 | 2 | 3 | 2 |
| 3 | 3 | 4 | 2√2 |
| 4 | 2 | 4 | 2 |
| 5 | 1 | 4 | 2√2 |
| 6 | 5 | 4 | 2 |

### MATLAB input

```matlab
nodes = [0 0; 2 0; 4 0; 2 2; 0 2];
members = [1 2; 2 3; 3 4; 2 4; 1 4; 5 4];
E = 200e9;
A = 0.0015;
fixedDOFs = [1 2 9 10];
F = zeros(10,1);
F(6) = -30000;
```

### Reference results

| Quantity | Result |
| --- | ---: |
| Joint 2 vertical displacement | −0.965685 mm — downward |
| Member 5 axial force | −42.426407 kN — compression |
| Member 1 axial force | −30.000 kN |
| Member 2 axial force | −30.000 kN |
| Member 3 axial force | +42.426407 kN |
| Member 4 axial force | Approximately 0 kN |
| Member 6 axial force | +60.000 kN |
| Node 1 reactions (Rx, Ry) | (+60.000, +30.000) kN |
| Node 5 reactions (Rx, Ry) | (−60.000, 0.000) kN |

Member 4 is a zero-force member for this particular loading. This does not imply that it is unnecessary for other load cases.

## Degrees of freedom and sign conventions

For node `i`, use:

```matlab
horizontalDOF = 2*i - 1;
verticalDOF   = 2*i;
```

Consequently, **the vertical displacement at joint 2 is `U(4)` in both exercises**. The original textbook uses a different DOF ordering; do not use its arrow numbers directly as MATLAB indices.

- Positive x is rightwards; positive y is upwards.
- Positive axial force indicates tension; negative axial force indicates compression.
- Internal units are metres, newtons and pascals.
- Printed results use millimetres, kilonewtons and megapascals.

## Calculation workflow

1. Define coordinates, connectivity, properties, loads and restraints.
2. Initialise the global stiffness matrix.
3. For each member, calculate geometry and element stiffness, then add its contribution to the global matrix.
4. Extract the reduced system for the free DOFs.
5. Solve the unknown displacements using MATLAB backslash.
6. Recover reactions, member axial forces and stresses.
7. Check symmetry, equilibrium and reference values.
8. Display result tables and the magnified deformed configuration.

For a member with direction cosines `c` and `s`:

```matlab
b = [-c -s c s];
ke = (E*A/L)*(b.'*b);
dofs = [2*i-1 2*i 2*j-1 2*j];
K(dofs,dofs) = K(dofs,dofs) + ke;
```

With zero prescribed support displacements:

```matlab
U(freeDOFs) = K(freeDOFs,freeDOFs) \ F(freeDOFs);
R = K*U - F;
N(e) = (E*A/L)*b*U(dofs);
```

## How to follow the tutorial

1. Download the repository files into one folder and select it as MATLAB's current folder.
2. Read the PDF introduction and identify nodes, members, loads and supports.
3. Open `exercise1_starter.m` and complete its TODO expressions.
4. Use the Exercise 1 Live Script to inspect intermediate calculations section by section. Run sections in order because later sections depend on earlier variables.
5. Compare your results with the reference tables and completed solution.
6. Apply the same method to `exercise2_starter.m` and check against `exercise2_solution.m`.

The starter scripts intentionally stop until their missing expressions are completed. Each completed solution is standalone.

### Suggested one-hour class sequence

| Time | Activity |
| --- | --- |
| 0–10 min | Review the model and enter Exercise 1 |
| 10–30 min | Complete geometry, stiffness, assembly and recovery expressions |
| 30–35 min | Verify the first solution |
| 35–50 min | Transfer the method to Exercise 2 |
| 50–60 min | Check results and discuss structural behaviour |

## Verification and exploration

The completed solutions check stiffness symmetry, free-DOF residuals, global force and moment equilibrium, and selected reference results. Numerical results have also been independently checked outside MATLAB; the supplied scripts were not executed in MATLAB during preparation.

Try doubling every member area and predict the outcome before running the model. For these stable, statically determinate trusses under unchanged nodal loads and zero support settlements, forces and reactions remain unchanged while displacements halve.

When changing the model, retain equilibrium checks but update or disable assertions tied to the original reference results. The explanatory Exercise 1 Live Script automatically skips its original reference checks when its baseline input changes.

## Assumptions and limitations

This is a first-order, linear elastic, two-dimensional truss model. It assumes ideal pinned joints, axial deformation only, nodal loading and zero support settlements. It neglects self-weight, thermal effects, bending, shear deformation and connection flexibility.

The scripts do not assess buckling or nonlinear response. Deformation plots are deliberately magnified; use the numerical tables for actual displacement values.

## Exercise sources

The geometry and problem statements are based on the supplied textbook excerpts **Example 14.3** and **Exercise 14–13**. The textbook author and edition were not provided. Node and member labels are retained, while MATLAB DOFs follow the consecutive-by-node convention described above.
