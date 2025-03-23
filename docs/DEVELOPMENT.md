# Development Guidelines for LiftLoggerSense

This document outlines the development practices and guidelines for contributing to the LiftLoggerSense project.

## Branch Strategy

We follow a simple branching strategy:

- `main`: Production-ready code that has been thoroughly tested
- `dev`: Development branch where new features are integrated before being merged to main

## Development Workflow

1. Create feature branches from `dev` branch
2. Implement and test your changes locally
3. Submit a pull request to merge into the `dev` branch
4. After review and approval, changes will be merged
5. Periodically, `dev` will be merged into `main` for release

## Coding Standards

- Follow ESP-IDF style guide for C/C++ code
- Use descriptive variable and function names
- Add comments for complex logic
- Keep functions small and focused on a single task

## Testing

All new features should include:
- Unit tests where applicable
- Integration testing with physical hardware before merging to main

## Commit Messages

Write clear, descriptive commit messages that explain:
- What changes were made
- Why they were necessary

Example: "Add temperature calibration for QMI8658C sensor to improve accuracy"