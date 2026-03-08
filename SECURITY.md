# Security Policy

## Supported Scope

This repository is a public robotics simulation project for `ROS1 Noetic` and `Gazebo 11`.

Security-sensitive reports may include:

- malicious or unsafe launch-time behavior introduced by scripts or plugins
- unsafe dependency changes
- embedded secrets or credentials committed by mistake
- configuration changes that create unintended exposure in shared research or lab environments

## Reporting a Vulnerability

Please do not post full exploit details in a public issue.

Instead, contact the maintainer privately first and include:

- a short description of the issue
- affected file or package
- reproduction steps
- impact assessment
- any logs or screenshots that help confirm the report

If no private contact channel is available yet, open a minimal public issue without sensitive details and ask for a private follow-up.

## Response Goals

The maintainer will try to:

- acknowledge the report
- confirm whether the issue is reproducible
- decide whether the fix should be handled privately first
- publish a fix or mitigation note when appropriate

## Operational Notes

- Do not commit API keys, tokens, private SSH material, or lab credentials.
- Review external dependencies before upgrading them.
- Re-check launch files and plugin configuration before publishing new releases.
