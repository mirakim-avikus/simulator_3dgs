# AvikuSim Development Guidelines

## Code Writing Principles
- Write concisely and clearly
- Use appropriate modularization (avoid splitting too finely or creating overly large files)
- Avoid unnecessary abstraction and excessive error handling
- Do NOT write test code - just report what was done to the user

## Documentation Management
- `docs/planning/`: Planning documents (keep concise and clear)
- `docs/CHANGELOG.md`: Development changelog

## Change Management
- Prefer incremental modifications over large structural changes
- When significant structural changes are required, notify the user before proceeding
- If a feature can be implemented by modifying existing code, avoid creating new files

## Workflow
1. Develop with reference to planning documents
2. Record in CHANGELOG.md upon completion
3. Update planning documents when implementation differs from plan