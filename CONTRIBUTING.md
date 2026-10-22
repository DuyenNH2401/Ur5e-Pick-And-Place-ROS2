# Contributing Guidelines

Thank you for your interest in contributing to the **UR5e Tray Workcell (ROS 2 & Gazebo)** project!

We welcome bug fixes, documentation improvements, performance optimizations, and feature enhancements.

---

## Code of Conduct

Please maintain a respectful, constructive, and collaborative tone in all discussions, issues, and pull requests.

---

## Development Setup

1. **System Requirements**:
   - Ubuntu 24.04 / 26.04
   - ROS 2 (Lyrical or Rolling)
   - Gazebo Jetty / Harmonic
   - Python 3.12+

2. **Clone & Install Dependencies**:
   ```bash
   git clone https://github.com/DuyenNH2401/Ur5e-Pick-And-Place-ROS2.git
   cd Ur5e-Pick-And-Place-ROS2

   # Install ROS 2 system dependencies
   rosdep install --from-paths src --ignore-src --rosdistro lyrical -y

   # Install Python tooling
   pip install -r requirements.txt
   ```

3. **Build the Workspace**:
   ```bash
   source /opt/ros/lyrical/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## Testing & Quality Assurance

Before submitting a Pull Request, ensure that:

1. **All unit tests pass**:
   ```bash
   source install/setup.bash
   python3 -m pytest tests -q
   ```

2. **Headless simulation verification passes**:
   ```bash
   python3 scripts/run_headless.py
   ```
   The script must finish with `PASS` and `completed: 6 / expected: 6`.

3. **Code formatting and style**:
   Follow PEP 8 guidelines. You can check and format using:
   ```bash
   ruff check .
   black --check .
   ```

---

## Submitting Pull Requests

1. Create a descriptive feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
2. Commit your changes with clear, semantic commit messages (e.g., `feat: ...`, `fix: ...`, `docs: ...`).
3. Ensure no generated build files (`build/`, `install/`, `log/`, `.runtime/`) are committed.
4. Open a Pull Request on GitHub against the `main` branch with a summary of changes and test evidence.
