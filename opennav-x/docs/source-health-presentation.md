# Source Health presentation

The prototype's **Know what to trust** sheet reads owned Vessel Data, selected
source metadata, onboard AIS reports, the supplemental AISStream connection and
measured pilot feedback. It cannot refresh observations, select a source,
connect a service or send a command. Opening a disclosure only changes the UI.

`application::PresentSourceHealth` distinguishes current, aging, stale,
estimated, uncertain, invalid and unavailable measurements. GPS requires the
latitude/longitude pair to share source, device identity and observation time,
with finite coordinates in range. A latitude alone does not establish GPS
health. Repeated reads retain the original age. Malformed freshness thresholds
are invalid rather than an exception in the painter.

The summary rows name representative measurements: Heading, Depth below the
transducer, apparent wind speed, motor RPM, battery SOC, rudder and fresh-water
level. Expanded details identify the measurement. A current motor RPM does not
mean its temperature or every value on the bus is current. Observed cadence and
priority appear only when exactly one selected registry entry matches the copied
measurement's source, device and observation time. No PGN or frequency is guessed
from a label. Advanced sensor selection remains in the existing settings flow.

Onboard AIS health describes received OpenCPN target reports, never presumed
receiver connectivity. Online-origin targets are excluded even if supplied in
the wrong input collection. **Online AIS** is a separate row, with connection
and subscription-confirmation state. It links to the user's protected key and
enable settings. Neither an internet connection nor received internet traffic
makes the onboard receiver healthy. No credential is part of this view.

Pilot health requires sequence-bearing, timestamped feedback and the accepted
three-second freshness boundary. Sending a command does not make status current.
Historical replay is visibly identified; live configuration entry points are
disabled. Diagnostic export remains an explicit action through the existing
field-report selection flow.

The native drawer uses the shared disclosure component, exact 398px prototype
sheet, 69.1875px collapsed sensor rows with cumulative rounding, 8px gaps/radii
and prototype text roles. It
reuses current source setup, AIS settings, pilot configuration and export actions.
It contains no mock-network or simulated dropout controls. Current visual and
platform acceptance is recorded separately; implementation is not conformance.
