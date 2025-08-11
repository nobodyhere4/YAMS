---
description: Welcome to your team’s developer platform
layout:
  width: wide
  title:
    visible: false
  description:
    visible: false
  tableOfContents:
    visible: false
  outline:
    visible: false
  pagination:
    visible: false
  metadata:
    visible: true
---

# Developer Platform

<h2 align="center">Yet Another Mechanism System</h2>

<p align="center">Welcome to your team’s new mechanism system</p>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th data-hidden data-card-target data-type="content-ref"></th><th data-hidden data-card-cover data-type="files"></th></tr></thead><tbody><tr><td><h4><i class="fa-leaf">:leaf:</i></h4></td><td><strong>Easy interface</strong></td><td>Get started with the developer platform in 5 minutes.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/">Documentation</a></td><td><a href=".gitbook/assets/no-code.jpg">no-code.jpg</a></td></tr><tr><td><h4><i class="fa-terminal">:terminal:</i></h4></td><td><strong>API reference</strong></td><td>Browse, test, and implement APIs.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ezOwaXLQ3h1N7tr3zYnj/">API Reference</a></td><td><a href=".gitbook/assets/api-reference.jpg">api-reference.jpg</a></td></tr></tbody></table>

{% columns %}
{% column width="33.33333333333333%" %}
### Get started in 5 minutes

Setting up your first API call should be the easiest part of getting started. With clear endpoints, copy-paste-ready examples, and quick authentication, you’ll be up and running in minutes—not hours.

No guesswork, no complexity—just your first successful call, fast.

<a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/" class="button primary" data-icon="rocket-launch">Get started</a> <a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ezOwaXLQ3h1N7tr3zYnj/" class="button secondary" data-icon="terminal">API reference</a>
{% endcolumn %}

{% column width="66.66666666666667%" %}
{% code title="ArmSubsystem.java" overflow="wrap" %}
```java
// Create your motor controller config
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(gearing(gearbox(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);

// Create your motor
TalonFXS                   armMotor    = new TalonFXS(1);
SmartMotorController       motor       = new TalonFXSWrapper(armMotor, DCMotor.getNEO(1), motorConfig);

// Create your mechanism config
ArmConfig                  m_config    = new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withStartingPosition(Degrees.of(0))
      .withHorizontalZero(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);
// Create mechanism
Arm                        arm         = new Arm(m_config);

public void periodic() { arm.updateTelemetry(); }
public void simulationPeriodic() { arm.simIterate(); }
public Command armCmd(double dutycycle) { return arm.set(dutycycle); }
public Command sysId() { return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30)); }
public Command setAngle(Angle angle) { return arm.setAngle(angle); }

```
{% endcode %}
{% endcolumn %}
{% endcolumns %}

{% columns %}
{% column %}
<figure><img src="https://gitbookio.github.io/onboarding-template-images/placeholder.png" alt=""><figcaption></figcaption></figure>
{% endcolumn %}

{% column %}
### Learn more about Yet Another Mechanism System

Read guides, watch tutorials, and learn more about working with the developer platform and integrating it with your own stack.

<a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/unoQcLkz5R1wNeAuLySo/" class="button primary" data-icon="book-open">Guides</a> <a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/" class="button secondary" data-icon="book">Documentation</a>
{% endcolumn %}
{% endcolumns %}

<h2 align="center">Join a community of over 400 teams!</h2>

<p align="center">Join our Discord community or create your first PR in just a few steps.</p>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th></th><th data-hidden data-card-cover data-type="files"></th></tr></thead><tbody><tr><td><h4><i class="fa-discord">:discord:</i></h4></td><td><strong>Discord community</strong></td><td>Join our Discord community to post questions, get help, and share resources with over 3,000 like-minded developers.</td><td><a href="https://discord.gg/yagsl" class="button secondary">Join Discord</a></td><td></td></tr><tr><td><h4><i class="fa-github">:github:</i></h4></td><td><strong>GitHub</strong></td><td>Our product is 100% open source and built by developers just like you. Head to our GitHub repository to learn how to submit your first PR.</td><td><a href="https://github.com/Yet-Another-Software-Suite/YAMS/pulls" class="button secondary">Submit a PR</a></td><td></td></tr></tbody></table>
