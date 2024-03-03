// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

/** Add your docs here. */
public class InterpolationOptimization {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    BooleanTopic su = nt.getBooleanTopic("arm/small change up");
    BooleanTopic sd = nt.getBooleanTopic("arm/small change down");
    BooleanTopic mu = nt.getBooleanTopic("arm/medium change up");
    BooleanTopic md = nt.getBooleanTopic("arm/medium change down");
    BooleanTopic lu = nt.getBooleanTopic("arm/large change up");
    BooleanTopic ld = nt.getBooleanTopic("arm/large change up");

    BooleanPublisher su_pub;
    BooleanPublisher sd_pub;
    BooleanPublisher mu_pub;
    BooleanPublisher md_pub;
    BooleanPublisher lu_pub;
    BooleanPublisher ld_pub;

    BooleanSubscriber su_sub;
    BooleanSubscriber sd_sub;
    BooleanSubscriber mu_sub;
    BooleanSubscriber md_sub;
    BooleanSubscriber lu_sub;
    BooleanSubscriber ld_sub;

    BooleanSubscriber[] subArr = { su_sub, sd_sub, mu_sub, md_sub, lu_sub, ld_sub };
    BooleanPublisher[] pubArr = { su_pub, sd_pub, mu_pub, md_pub, lu_pub, ld_pub };
    // boolean[] schedule = new boolean[6];

    public InterpolationOptimization() {

        su_pub = su.publish();
        su_pub.setDefault(false);
        su_sub = su.subscribe(false, PubSubOption.sendAll(true));

        sd_pub = sd.publish();
        su_pub.setDefault(false);
        su_sub = su.subscribe(false, PubSubOption.sendAll(true));

        mu_pub = mu.publish();
        mu_pub.setDefault(false);
        mu_sub = mu.subscribe(false, PubSubOption.sendAll(true));

        md_pub = md.publish();
        mu_pub.setDefault(false);
        mu_sub = mu.subscribe(false, PubSubOption.sendAll(true));

        lu_pub = lu.publish();
        lu_pub.setDefault(false);
        lu_sub = lu.subscribe(false, PubSubOption.sendAll(true));

        ld_pub = ld.publish();
        lu_pub.setDefault(false);
        lu_sub = lu.subscribe(false, PubSubOption.sendAll(true));
    }

    public int scheduleFunctionChanges() {
        for (int i = 0; i < subArr.length; i++) {
            // schedule[i] = subArr[i].get();
            if (subArr[i].get()) {
                pubArr[i].set(false);
                return i;
            }
        }

        return -1;
    }
}
