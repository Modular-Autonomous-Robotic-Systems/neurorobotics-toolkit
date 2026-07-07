# Neurorobotics Workspace (NRT_WS) Guidelines

## Commit Message Conventions

To maintain a clean and informative project history, all commits must adhere to the following strict guidelines.

### Structure

The commit message should be structured with a subject line, followed by `Problem`, `Solution`, and `Note` sections.

```text
<Subject Line>

Problem
=======
<Brief description of the issue or motivation (max 3 lines)>

Solution
========
<Detailed list of changes>

Note
====
<Additional context, breaking changes, or deletions>
```

### Style Rules

1.  **Subject Line**:
    *   Imperative mood (e.g., "Fix bug" not "Fixed bug").
    *   Capitalized start, no trailing period.
    *   **Max 50 characters**.
    *   Blank line after the subject.

2.  **Content**:
    *   **Wrap at 72 characters**.
    *   **Method References**: Must include the class scope (e.g., `LifecycleControllerBase::SyncCallChangeState`).
    *   **File Paths**: Must use the full relative path from the repo root (e.g., `src/controllers/include/controllers/common/controller.h`).

3.  **Sections**:
    *   **Problem**: *What* is broken or missing and *why* it needs fixing.
    *   **Solution**: *What* specific changes were made to address the problem.
    *   **Note**: Deletions, side effects, or special instructions.
