# Neurorobotics Workspace (NRT_WS) Guidelines

## Commit Message Conventions

To maintain a clean and informative project history, all commits must adhere to the following strict guidelines. Furthermore, the commit message must be prompted to the user for validation before actually committing anything. Commit messages need to be derived from git diff and must list down all the changes in the diff, the rationale behind the changes and a high level description of the changes as explained below.

### Structure

The commit message should be structured with a subject line, followed by `Problem`, `Solution`, and `Note` sections.

```text
<Subject Line>

Problem
=======
1. <First issue or motivation>
2. <Second issue or motivation>

Solution
========
1. <First change, naming the symbols and files it touched>
2. <Second change, naming the symbols and files it touched>

Note
====
1. <Additional context, breaking change, or deletion>
```

All three sections carry numbered lists. Every entry is one number, even where a section holds a single entry. `Note` may be left with its heading and no entries when there is nothing to record.

### Style Rules

1.  **Subject Line**:
    *   A noun phrase naming the change (e.g., "QoS Mismatch Fix for Image Topic Subscribers", "VIO Integration"). Correction, 2026-08-23. This rule previously required the imperative mood ("Fix bug" not "Fixed bug"). The repository history uses noun phrases throughout, so the imperative rule described no commit actually written and has been replaced.
    *   Capitalized start, no trailing period.
    *   **Max 50 characters**.
    *   Blank line after the subject.

2.  **Content**:
    *   **Wrap at 72 characters**.
    *   **Numbered entries**: Every entry in every section is numbered. Continuation lines are indented to align under the text of their entry, which is three spaces for single digit numbers.
    *   **Short sentences**: One claim per sentence. Split a compound sentence rather than joining it with a subordinate clause. Per point must be as few sentences as possible. Do not delve into details which are evident from the code. The commit message must provide a high level overview of the change made, files updated and a short rationale. Do not cross more than 12 words and 2 sentences per point
    *   **Backticks**: Every file path, class name, method name, member variable, type, macro, enumerator and build flag is wrapped in backticks. This covers prose in all three sections, not only the symbol being changed.
    *   **Method References**: Must include the class scope (e.g., `LifecycleControllerBase::SyncCallChangeState`).
    *   **Class References**: May be written with the `class` keyword where the class itself is the subject (e.g., `class BasaltSLAMNode`).
    *   **File Paths**: Must use the full relative path from the repo root (e.g., `src/controllers/include/controllers/common/controller.h`).
    *   **No AI authoring credentials**: Never append `Co-Authored-By`, `Claude-Session`, `Generated with` or any equivalent trailer. The author owns the commit.

3.  **Sections**:
    *   **Problem**: *What* is broken or missing and *why* it needs fixing.
    *   **Solution**: *What* specific changes were made to address the problem and *why*. Each entry names the symbols introduced or modified and the file each one lives in followed by the reason for the changes made depicting the rationale or design decision here.
    *   **Note**: Deletions, side effects, or special instructions. Documentation updates, auto-formatting sweeps and known defects left standing belong here.
