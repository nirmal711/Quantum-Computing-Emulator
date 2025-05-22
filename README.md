# Quantum Computing Emulator – Hardware Design

This project implements a **hardware-based emulator** for quantum computations using complex-valued matrix multiplications. Developed as part of **ECE 564 (Fall 2023)** under **Dr. Paul Franzon** at **NC State University** as a **volunteer intern** project.

## 🧠 Project Overview

The emulator simulates quantum operations by multiplying a quantum state vector with a sequence of quantum gate matrices. It processes complex-valued matrices using floating-point arithmetic, stores intermediate results in scratchpad SRAM, and writes the final result to output SRAM.

## ⚙️ Key Features

- Complex matrix multiplication pipeline with FP units
- Sequential quantum gate application (each output feeds the next)
- SRAM-based storage (initial state, scratchpad, and result)
- FSM-controlled operation for efficient compute and data routing

## 📊 Performance

- **Clock Period:** 8.7 ns  
- **Logic Area:** 23,791.839 µm²  
- **Tech Library:** `NangateOpenCellLibrary_PDKv1_2_v2008_10_slow_nldm.db`

## 🛠️ Optimization Summary

- Reduced FP units from 4 to 2 (1× `DW_fp_mult`, 1× `DW_fp_add`)
- Unified operand registers controlled via FSM
- Reduced FSM complexity through signal reuse

---

## 🚀 How to Run

### 1. Unzip the Project

```bash
unzip ECE564_Project.zip
cd ECE564_Project
```
### 2. Run Simulation

Go to the run/ directory
To run with UI (debug mode):
```bash
make debug
```
To run all grading tests (ECE 564 test suite):
```bash
make grade-564
```
### 3. Run Synthesis
Go to the synthesis/ directory:
Run synthesis with specified clock period:
```bash
make all CLOCK_PER=8.6
```
