package org.firstinspires.ftc.teamcode.PurePursuit;

import java.util.ArrayList;

public class Path {
    public final ArrayList<WaypointGenerator> waypoints;
    public final double timeout; // In milliseconds

    public Path(double timeout, WaypointGenerator... waypoints) {
        this.waypoints = new ArrayList<>();
        for (WaypointGenerator waypoint : waypoints) {
            this.waypoints.add(waypoint);
        }
        this.timeout = timeout;
    }

    public Path(WaypointGenerator... waypoints) {
        this(Double.POSITIVE_INFINITY, waypoints);
    }

    private Path(Builder builder) {
        waypoints = builder.waypoints;
        timeout = builder.timeout;
    }

    public static class Builder {
        private final ArrayList<WaypointGenerator> waypoints;
        private double defaultRadiusIn = 15.0; // Default follow radius in cm
        private double defaultMaxVelocity = Double.POSITIVE_INFINITY;
        private double timeout = Double.POSITIVE_INFINITY;

        private Builder() {
            waypoints = new ArrayList<>();
        }

        public Builder addWaypoint(WaypointGenerator waypoint) {
            waypoints.add(waypoint);
            return this;
        }

        public Builder setDefaultRadius(double defaultRadiusIn) {
            this.defaultRadiusIn = defaultRadiusIn;
            return this;
        }

        public Builder setDefaultMaxVelocity(double defaultMaxVelocity) {
            this.defaultMaxVelocity = defaultMaxVelocity;
            return this;
        }

        public Builder addWaypoint(double x, double y) {
            waypoints.add(new Waypoint(x, y, defaultRadiusIn, defaultMaxVelocity));
            return this;
        }

        public Builder join(Path path) {
            path.waypoints.forEach((WaypointGenerator waypoint) -> waypoints.add(waypoint));
            return this;
        }

        public Builder setTimeout(double timeout) {
            this.timeout = timeout;
            return this;
        }

        public Path build() {
            return new Path(this);
        }
    }

    public Waypoint[] generateWaypoints() {
        ArrayList<Waypoint> generatedWaypoints = new ArrayList<>();
        waypoints.forEach((WaypointGenerator waypoint) -> generatedWaypoints.add(waypoint.getWaypoint()));
        return generatedWaypoints.toArray(new Waypoint[]{});
    }

    public Waypoint[][] generateLineSegments() {
        ArrayList<Waypoint[]> segments = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            segments.add(new Waypoint[]{waypoints.get(i).getWaypoint(), waypoints.get(i + 1).getWaypoint()});
        }
        return segments.toArray(new Waypoint[][]{});
    }

    public static Builder getBuilder() {
        return new Builder();
    }

    public double getTimeout() {
        return timeout;
    }

    public Path appendWaypoint(WaypointGenerator waypoint, double timeout) {
        return join(new Path(timeout, waypoint));
    }

    public Path join(Path path) {
        return join(path, path.timeout + timeout);
    }

    /**
     * Creates a new path by joining the specified path to the end of this path instance.
     */
    public Path join(Path path, double timeout) {
        Builder builder = getBuilder().setTimeout(timeout);
        for (WaypointGenerator waypoint : waypoints) {
            builder.addWaypoint(waypoint);
        }
        for (WaypointGenerator waypoint : path.waypoints) {
            builder.addWaypoint(waypoint);
        }
        return builder.build();
    }
}