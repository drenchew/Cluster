#include "Application.h"

#include <Arduino.h>

Application::Application()
    : cluster_(bus_), console_(cluster_, demo_, host_) {
}

void Application::begin() {
    Serial.begin(SerialBaud);
    Serial.setTimeout(cfg::console::ReadTimeoutMs);
    delay(StartupDelayMs);

    if (!cluster_.begin()) {
        Serial.println(F("CAN init FAILED - check SPI wiring and 5 V on the HW-184. Halting."));
        while (true) {
            delay(HaltRetryMs);
        }
    }

    cluster_.state().setEngineRunning(cfg::limits::EngineRunningAtBoot);
    cluster_.state().setLightMask(cfg::lights::SideLight | cfg::lights::LowBeam);

    // A cluster taken from a scrapped vehicle usually powers up showing a
    // service message, which this jig has no service data to justify.
    if (cfg::symbol::SuppressServiceAtBoot) {
        cluster_.setSymbol(cfg::symbol::Service, false);
    }

    console_.printBanner();
}

void Application::updateControlSource() {
    const bool live = host_.hostLive();
    if (live != hostWasLive_) {
        hostWasLive_ = live;
        if (live) {
            demo_.stop();
            Serial.println(F("host connected"));
        } else {
            demo_.resume();
            // Nothing the host last requested is necessarily true any more.
            // A telltale left showing because the host closed mid-event would
            // otherwise stay lit for as long as the board is powered.
            cluster_.clearDriverRequests();
            Serial.println(F("host lost - demo sweep"));
        }
    }
}

void Application::updatePeriodicReports() {
    const uint32_t now = millis();

    // Reported whether or not the host is driving, because the host's log is
    // where this is read from.
    if (cfg::feedback::Enabled && (now - lastFeedbackMs_) >= cfg::feedback::ReportPeriodMs) {
        lastFeedbackMs_ = now;
        console_.printClusterFeedback();
    }

    if ((now - lastStatusMs_) >= StatusPeriodMs) {
        lastStatusMs_ = now;
        if (!hostWasLive_) {
            console_.printStatus();
        }
    }
}

void Application::run() {
    console_.poll();

    // The host owns the setpoints while it is alive and the local sweep takes
    // over as soon as it stops, so the dials never freeze on the last value
    // received and the cluster always sees a plausible running engine.
    updateControlSource();

    demo_.update(cluster_);
    cluster_.update();

    updatePeriodicReports();
}
