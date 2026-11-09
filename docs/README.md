# DroneNode documentation

Reviewed: 2026-11-06. Role: ESP32 firmware.

- [Local architecture](architecture.md)
- [Local operations and serial discovery runbook](operations.md)
- [System documentation index](../../DroneLinkDatastore/docs/README.md)
- [Critical system summary](../../DroneLinkDatastore/docs/SUMMARY.md)
- [Shared packet/API contracts](../../DroneLinkDatastore/docs/contracts/README.md)
- [Master roadmap and implementation plans](../../DroneLinkDatastore/docs/planning/roadmap.md)
- [Documentation conventions](../../DroneLinkDatastore/docs/contributing.md)

System facts and roadmap status are maintained in Datastore; this repo owns its component details. Cross-repo links assume sibling checkouts with these names.

## Project intent and accepted target boundaries

Read [objectives and requirements](../../DroneLinkDatastore/docs/project/objectives.md), [traceability](../../DroneLinkDatastore/docs/project/traceability.md) and [target repository architecture](../../DroneLinkDatastore/docs/architecture/target.md). The target responsibilities are accepted; firmware/artifact and bounded bench evidence are recorded in SEP-05/SEP-07. The local architecture guide describes current source.

## Architecture baseline

The current cross-repository direction is frozen as [ARCH-1.1](../../DroneLinkDatastore/docs/architecture/baseline.md). Use the [canonical glossary](../../DroneLinkDatastore/docs/project/glossary.md) for actors and terms, and [requirements](../../DroneLinkDatastore/docs/project/requirements.md) for policy. Completed [SEP-01](../../DroneLinkDatastore/docs/planning/sep-01-protocol-compatibility.md) is the bounded compatibility foundation; ARCH-1.1 access/domain policy introduces no firmware change.

ARCH-1.1 publication (2026-11-06): accepted [ADR-0004](../../DroneLinkDatastore/docs/decisions/0004-cloud-access-and-domain-retirement.md) defines managed-cloud access/domain retirement; [M1 slice A](../../DroneLinkDatastore/docs/planning/m1-contracts-and-telemetry.md#slice-a-implementation-plan--2026-11-06) is ready, runtime implementation unstarted. The local annotated `architecture-v1.1` tag captures this documentation adoption; the original `architecture-v1.0` snapshot is retained unchanged. This component documentation change modifies no runtime, deployment, account or firmware.

CloudTelemetry G.1 is locally implemented/tested; its [architecture](architecture.md#cloudtelemetry--g1-local-implementation) and [configuration/verification](operations.md#cloudtelemetry-g1-configuration-and-local-verification) document the opt-in producer. G.2 packed Server integration and G.3 clean-source exports are locally verified; the [release checkpoint](operations.md#m1-g-g3-clean-release-checkpoint--2026-11-09) records exact artifacts. Physical/managed acceptance remains G.4.
