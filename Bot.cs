// A robot on the arena's shared .NET core binding.
// The core owns tracking, geometry, parsing and act serialization.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
using System;
using Clunkyard;

class Bot : Robot {
    bool free;
    double Steer(double towards,double current)=>Angles.Clamp(Angles.AngleDiff(towards,current)/45);
    public override void OnStart(){free=Match.Is3d && Arena.Gravity.Z==0;}
    public override void OnTurn(){
        var self=Me;var position=self.Position;var gun=Gun;var scanner=Scanner;
        Drive.Forces(brake:0.2);
        var target=Sensors.Nearest;var cue=Tracker.Cue;
        if(target!=null){
            double throttle=target.Range>350?0.7:target.Range<200?-0.4:0.2;
            Drive.Set(throttle,Steer(target.Bearing,self.Heading),Steer(target.Elevation,self.Pitch));
            scanner.Set(target.Bearing,20,target.Elevation);
            double power=gun.PowerForFlightTime(target.Range,1);var aim=gun.Lead(target,power);
            double heading=aim?.Heading??target.Bearing,pitch=aim?.Pitch??target.Elevation;
            gun.Aim(heading,pitch);var now=gun.AimNow;
            if(Math.Abs(Angles.AngleDiff(heading,now.Heading))<3 && Math.Abs(pitch-now.Pitch)<3 && gun.CanFire(power,10))gun.Fire(power);
            Status.Set("TRACK",target.Range);
        }else if(cue!=null){
            double bearing=Angles.BearingTo(position,cue.Position),distance=Angles.Distance(position,cue.Position);
            double elevation=free?Angles.ElevationTo(position,cue.Position):0,error=Math.Min(cue.BearingError,30);
            int tick=Clock.Tick;double across=(tick%5-2)/2.0*error,up=free?((tick/5)%3-1)*error/2:0;
            Drive.Set(0.7,Steer(bearing,self.Heading),Steer(elevation,self.Pitch));
            scanner.Set(bearing+across,scanner.ArcForReach(distance+100),elevation+up);Status.Set("CUE",distance);
        }else{
            Drive.Set(0.5,0.33,Steer(0,self.Pitch));double pitch=free?((Clock.Tick/10)%3-1)*35:0;
            scanner.Set(Angles.Normalize(scanner.Last.Heading+36),40,pitch);Status.Set("SEARCH");
        }
        if(Battery.Charge<Battery.Capacity/4)Drive.Set(throttle:(Drive.Get("throttle")??0)*0.5);
        var storm=Nav.Storm;
        if(storm!=null && (storm.Margin(position)??-1e9)>-60){
            double away=Angles.BearingTo(storm.Center,position);
            Drive.Set(throttle:0.8,turn:Angles.Clamp(Angles.AngleDiff(away,self.Heading)/30));
        }
        var zone=Nav.Zone;
        if(zone!=null && (zone.Margin(position)??1e9)<60){
            double climb=zone.Shape==Shape.Sphere?Angles.ElevationTo(position,zone.Center):0;
            Drive.Set(0.8,Angles.Clamp(Angles.AngleDiff(Angles.BearingTo(position,zone.Center),self.Heading)/30),Angles.Clamp((climb-self.Pitch)/30));
        }
    }

    static void Main()=>Clunkyard.Clunkyard.Run(new Bot());
}
