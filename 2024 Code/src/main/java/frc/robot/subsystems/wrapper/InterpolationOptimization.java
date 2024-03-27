// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrapper;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

/** Add your docs here. */
public class InterpolationOptimization {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    BooleanTopic su = nt.getBooleanTopic("optimizer/small change up");
    BooleanTopic sd = nt.getBooleanTopic("optimizer/small change down");
    BooleanTopic mu = nt.getBooleanTopic("optimizer/medium change up");
    BooleanTopic md = nt.getBooleanTopic("optimizer/medium change down");
    BooleanTopic lu = nt.getBooleanTopic("optimizer/large change up");
    BooleanTopic ld = nt.getBooleanTopic("optimizer/large change down");

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

    BooleanSubscriber[] subArr = new BooleanSubscriber[6];
    BooleanPublisher[] pubArr = new BooleanPublisher[6];
    // boolean[] schedule = new boolean[6];

    public InterpolationOptimization() {

        su_pub = su.publish();
        su_pub.setDefault(false);
        su_sub = su.subscribe(false, PubSubOption.sendAll(true));

        sd_pub = sd.publish();
        sd_pub.setDefault(false);
        sd_sub = sd.subscribe(false, PubSubOption.sendAll(true));

        mu_pub = mu.publish();
        mu_pub.setDefault(false);
        mu_sub = mu.subscribe(false, PubSubOption.sendAll(true));

        md_pub = md.publish();
        md_pub.setDefault(false);
        md_sub = md.subscribe(false, PubSubOption.sendAll(true));

        lu_pub = lu.publish();
        lu_pub.setDefault(false);
        lu_sub = lu.subscribe(false, PubSubOption.sendAll(true));

        ld_pub = ld.publish();
        ld_pub.setDefault(false);
        ld_sub = ld.subscribe(false, PubSubOption.sendAll(true));

        subArr[0] = su_sub;
        subArr[1] = sd_sub;
        subArr[2] = mu_sub;
        subArr[3] = md_sub;
        subArr[4] = lu_sub;
        subArr[5] = ld_sub;

        pubArr[0] = su_pub;
        pubArr[1] = sd_pub;
        pubArr[2] = mu_pub;
        pubArr[3] = md_pub;
        pubArr[4] = lu_pub;
        pubArr[5] = ld_pub;

        // pubArr = { su_pub, sd_pub, mu_pub, md_pub, lu_pub, ld_pub };
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
