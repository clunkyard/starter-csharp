// A robot on the arena's shared .NET core binding.
// The core owns tracking, geometry, parsing and act serialization.
// Bearing is earned: a scan contact's bearing is off by up to its
// BearingError (degrees, a fixed share of the arc), so a wide beam finds a
// robot and a narrow one aims at it. The robot narrows the beam, holds it
// on the target and aims along a line fitted through its recent sightings.
// Contacts within 120 units are exact.
// Clunkyard robot API 2.1.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude wall_contact hurt applied hull_mounts
//   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// drone: has ready in_flight launches_left report sightings
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   drone.launch(heading pitch param) drone.brief(index value)
//   drone.program(language source) status.set(text value) debug.log(line)
//   raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
using System;
using System.Linq;
using Clunkyard;

class Bot : Robot {
    const double SearchArc=40;  // degrees: sweeps the circle in half a second
    const double HoldWidth=60;  // units across at the target: it stays in the beam from tick to tick
    const double MinArc=6;      // degrees: the narrowest the beam holds
    const int Window=12;        // ticks of sightings the aim averages
    const int Sightings=8;      // narrow sightings in the window before a shot
    const int LookEvery=40;     // ticks between looks around the target, for a nearer robot
    const int Reacquire=10;     // ticks to look again where a lost target was

    bool free;
    int dodges,dodgeEnd; // dodges taken, and the tick the current one ends
    int slot;            // the last target's slot, for looking again where it was lost
    double Steer(double towards,double current)=>Angles.Clamp(Angles.AngleDiff(towards,current)/45);
    public override void OnStart(){free=Match.Is3d && Arena.Gravity.Z==0;}

    // A straight line through the narrowest of the last Window ticks' sightings: where the
    // robot is now and how fast it moves. Each sighting is off by its BearingError; the line averages that out.
    (Vec3 position,Vec3 velocity,int count)? Fit(int slot,int tick){
        var recent=Tracker.History(slot).Where(s=>s.Tick>tick-Window).ToList();
        if(recent.Count<2)return null;
        double best=recent.Min(s=>s.BearingError);
        var seen=recent.Where(s=>s.BearingError<=1.5*best).ToList(); // the narrow ones
        int n=seen.Count;if(n<2)return null;
        double t0=seen.Average(s=>(double)s.Tick),spread=seen.Sum(s=>(s.Tick-t0)*(s.Tick-t0));
        // One axis: the line's position now and its slope per tick.
        (double now,double rate) Axis(Func<Vec3,double> of){
            double mean=seen.Average(s=>of(s.Position)),slope=seen.Sum(s=>(s.Tick-t0)*(of(s.Position)-mean))/spread;
            return (mean+slope*(tick-t0),slope*Match.TickRate);
        }
        var x=Axis(p=>p.X);var y=Axis(p=>p.Y);var z=Axis(p=>p.Z);
        return (new Vec3(x.now,y.now,z.now),new Vec3(x.rate,y.rate,z.rate),n);
    }

    public override void OnTurn(){
        var self=Me;var position=self.Position;var gun=Gun;var scanner=Scanner;int tick=Clock.Tick;
        Drive.Forces(brake:0.2);
        var target=Sensors.Nearest;var cue=Tracker.Cue;
        int? lastSeen=slot!=0?Tracker.LastSeen(slot):null;
        if(target!=null){
            slot=target.Slot;
            double throttle=target.Range>350?0.7:target.Range<200?-0.4:0.2;
            Drive.Set(throttle,Steer(target.Bearing,self.Heading),Steer(target.Elevation,self.Pitch));
            // Halve the beam each tick until it is HoldWidth across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            double hold=Math.Max(MinArc,2*Math.Atan2(HoldWidth/2,target.Range)*180/Math.PI);
            // In a crowd a narrow beam sees one robot. Every LookEvery ticks,
            // widen it for a tick: a nearer robot it finds becomes the target.
            double arc=tick%LookEvery==0?SearchArc:Math.Max(hold,Math.Min(SearchArc,scanner.Last.Arc/2));
            scanner.Set(target.Bearing,arc,target.Elevation);
            double power=gun.PowerForFlightTime(target.Range,1);
            // An exact contact: gun.Lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            Solution? aim;bool ready=true;
            if(target.BearingError>0){
                var fit=Fit(target.Slot,tick);
                aim=fit is null?null:gun.LeadPoint(fit.Value.position,fit.Value.velocity,power);
                ready=fit is not null && fit.Value.count>=Sightings;
            }else aim=gun.Lead(target,power);
            double heading=aim?.Heading??target.Bearing,pitch=aim?.Pitch??target.Elevation;
            gun.Aim(heading,pitch);var now=gun.AimNow;
            if(ready && Math.Abs(Angles.AngleDiff(heading,now.Heading))<3 && Math.Abs(pitch-now.Pitch)<3 && gun.CanFire(power,10))gun.Fire(power);
            Status.Set(ready?"TRACK":"NARROW",target.Range);
        }else if(lastSeen is int seenAt && tick-seenAt<=Reacquire){
            // Lost it: look again where it was last seen, widening the beam, before searching.
            var where=Tracker.Position(slot)??position;
            double bearing=Angles.BearingTo(position,where),elevation=free?Angles.ElevationTo(position,where):0;
            Drive.Set(0.5,Steer(bearing,self.Heading),Steer(elevation,self.Pitch));
            scanner.Set(bearing,Math.Min(SearchArc,scanner.Last.Arc*2),elevation);Status.Set("REACQUIRE");
        }else if(cue!=null){
            double bearing=Angles.BearingTo(position,cue.Position),distance=Angles.Distance(position,cue.Position);
            double elevation=free?Angles.ElevationTo(position,cue.Position):0,error=Math.Min(cue.BearingError,30);
            double across=(tick%5-2)/2.0*error,up=free?((tick/5)%3-1)*error/2:0;
            Drive.Set(0.7,Steer(bearing,self.Heading),Steer(elevation,self.Pitch));
            scanner.Set(bearing+across,scanner.ArcForReach(distance+100),elevation+up);Status.Set("CUE",distance);
        }else{
            Drive.Set(0.5,0.33,Steer(0,self.Pitch));double pitch=free?((tick/10)%3-1)*35:0;
            scanner.Set(Angles.Normalize(scanner.Last.Heading+36),SearchArc,pitch);Status.Set("SEARCH");
        }
        // A simple dodge: vary it, or predict the shooter.
        if(tick>=dodgeEnd && Sensors.Cues.Any(c=>c.Source==Source.Muzzle)){dodges++;dodgeEnd=tick+10;}
        if(tick<dodgeEnd){
            // Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
            bool left=dodges%2==1;
            if(Match.Control==Control.Forces){Drive.Set(throttle:0.5);Drive.Forces(strafe:left?-1:1);}
            else Drive.Set(1,left?1:-1);
            Status.Set(left?"dodge:left":"dodge:right",dodges);
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
