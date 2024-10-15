//I did not receive nor give unauthorized aid on this project. Colin Farley
import java.io.InputStream;
import java.util.*;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in); //scanner to capture users input
        System.out.print("Enter a filename: "); //prompt
        String filename = scan.nextLine();
        RoadNetwork graph = readGraph(filename);  //read the graph from the file

        while (true) {
            System.out.print("\nEnter a start location ID or 0 to quit: "); //break sequence
            long startId = Long.parseLong(scan.nextLine()); //captures first ID
            if (startId == 0) break;

            System.out.print("Enter a goal location ID: ");
            long goalId = Long.parseLong(scan.nextLine()); //captures goal ID

            Location start = graph.getLocationForId(startId); //gets first location from ID
            Location goal = graph.getLocationForId(goalId); //gets goal location from ID

            if (start == null || goal == null) { //preventing errors
                System.out.println("Invalid location IDs. Please try again.");
                continue;
            }

            System.out.print("Do you want debugging information (y/n)? ");
            boolean debug = scan.nextLine().equalsIgnoreCase("y");

            System.out.print("Enter the number of times allowed to speed: ");
            int speedLimit = Integer.parseInt(scan.nextLine());

            List<Location> path = aStar(graph, start, goal, speedLimit, debug);  //calls A* search function

            if (path == null) {
                System.out.println("No path found between the given locations.");
            } else {
                System.out.println("Path found:"); //lists found path
                for (Location loc : path) {
                    System.out.println("Location ID: " + loc.id());
                }
            }
        }
        scan.close();
    }

    public static RoadNetwork readGraph(String filename) {
        InputStream is = Main.class.getResourceAsStream(filename); //gets inputted file
        if (is == null) {
            System.err.println("Bad filename: " + filename); //error handling
            System.exit(1);
        }

        Scanner scan = new Scanner(is);
        RoadNetwork graph = new RoadNetwork(); //initialize road network

        while (scan.hasNextLine()) {
            String[] pieces = scan.nextLine().split("\\|"); //parses the inputted file

            //slits up all the different data points from each line in the file
            if (pieces[0].equals("location")) {
                graph.addLocation(new Location(Long.parseLong(pieces[1]), Double.parseDouble(pieces[2]), Double.parseDouble(pieces[3])));
            } else if (pieces[0].equals("road")) {
                long startId = Long.parseLong(pieces[1]);
                long endId = Long.parseLong(pieces[2]);
                int speed = Integer.parseInt(pieces[3]);
                String name = pieces[4];
                graph.addRoad(new Road(startId, endId, speed, name));
                graph.addRoad(new Road(endId, startId, speed, name));  //add road in both directions
            }
        }

        scan.close();
        return graph;
    }

    private static void printDirections(List<Location> path, Map<Location, Road> roadToLocation, RoadNetwork graph, double totalTime) {
        System.out.println("\nTotal travel time in seconds: " + totalTime);
        System.out.println("Route found is:");

        for (Location loc : path) {
            System.out.println(loc.id() + (loc == path.get(0) ? " (starting location)" : ""));
        }

        System.out.println("\nGPS directions:");
        for (int i = 1; i < path.size(); i++) {
            Location from = path.get(i - 1);
            Location to = path.get(i);
            Road road = roadToLocation.get(to);

            double distance = Geometry.getDistanceInMiles(from, to);
            double time = Geometry.getDriveTimeInSeconds(road, graph);

            System.out.println("Head " + getCompassDirection(from, to) + " for " +
                    String.format("%.2f", distance) + " miles (" + String.format("%.2f", time) + " seconds)");
        }
        System.out.println("You have arrived!");
    }

    public static List<Location> aStar(RoadNetwork graph, Location start, Location goal, int speedLimit, boolean debug) {
        PriorityQueue<Node> frontier = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f)); //min-heap priority queue
        Map<Location, Double> reached = new HashMap<>(); //store min-cost to get to location
        Map<Location, Road> roadToLocation = new HashMap<>(); //tracks the road that leads to location
        Set<Node> visited = new HashSet<>(); //tracks visited nodes
        int visitedNodes = 0;

        //adds starting node to the frontier
        frontier.add(new Node(start, null, 0, heuristic(start, goal, speedLimit), speedLimit));
        reached.put(start, 0.0); //adds to reached map

        while (!frontier.isEmpty()) {
            Node currentNode = frontier.poll(); //get node with lowest f cost
            visitedNodes++;

            if (currentNode.location.equals(goal)) {
                System.out.println("Visited nodes: " + visitedNodes);
                List<Location> path = constructPath(currentNode); //if goal is reached, build path
                printDirections(path, roadToLocation, graph, currentNode.g); //print final path
                return path;
            }

            if (visited.contains(currentNode)) continue; //skip if already visited
            visited.add(currentNode);

            for (Road road : graph.getAdjacentRoads(currentNode.location.id())) { //get roads next to
                Location neighbor = graph.getLocationForId(road.endId()); //get their location
                double gNormal = currentNode.g + Geometry.getDriveTimeInSeconds(road, graph); //cost of driving (no speeding)

                //add successor node for normal driving
                addSuccessor(graph, frontier, currentNode, neighbor, gNormal, road, false, reached, roadToLocation, goal, speedLimit, debug);

                //if speeding is allowed
                if (currentNode.speedCount > 0) {
                    double gSpeed = currentNode.g + Geometry.getDriveTimeInSeconds(road, graph) / 2;
                    //add successor node for speeding
                    addSuccessor(graph, frontier, currentNode, neighbor, gSpeed, road, true, reached, roadToLocation, goal, currentNode.speedCount - 1, debug);
                }
            }
        }

        return null;
    }

    private static void addSuccessor(RoadNetwork graph, PriorityQueue<Node> frontier, Node currentNode, Location neighbor,
                                     double gCost, Road road, boolean speeding, Map<Location, Double> reached,
                                     Map<Location, Road> roadToLocation, Location goal, int speedCount, boolean debug) {

        double h = heuristic(neighbor, goal, speedCount); //estimates cost to reach the goal
        Node neighborNode = new Node(neighbor, currentNode, gCost, h, speedCount); //creates node for neighbor

        if (!reached.containsKey(neighbor) || gCost < reached.get(neighbor)) { //update frontier if new node is better
            frontier.add(neighborNode);
            reached.put(neighbor, neighborNode.f); //store new cost
            roadToLocation.put(neighbor, road); //track road

            if (debug) {
                System.out.println("Adding to frontier [State=" + neighbor.id() + ", parent=" + currentNode.location.id() +
                        ", g=" + gCost + ", h=" + h + ", f=" + neighborNode.f + ", action=" + (speeding ? "speeding" : "normal") + "]");
            }
        }
    }

    //determines cardinal directions for roads
    private static String getCompassDirection(Location from, Location to) {
        double latDiff = to.latitude() - from.latitude();
        double lonDiff = to.longitude() - from.longitude();

        return Math.abs(latDiff) > Math.abs(lonDiff) ? (latDiff > 0 ? "north" : "south") : (lonDiff > 0 ? "east" : "west");
    }


    private static double heuristic(Location current, Location goal, int speedCount) {
        double straightLineDistance = Geometry.getDistanceInMiles(current, goal); //estimates distance between current node and goal
        return (straightLineDistance / (65 * (speedCount > 0 ? 2 : 1))) * 3600; //estimates time based on speed
    }

    private static List<Location> constructPath(Node node) {
        List<Location> path = new ArrayList<>(); //holds final path (in reverse order)
        while (node != null) { //goes from goal to start, adding each node to the path
            path.add(node.location);
            node = node.parent;
        }
        Collections.reverse(path); //reverses it back to normal order
        return path;
    }
}

class Node {
    Location location;
    Node parent;
    double g; //cost to reach this node
    double h; //heuristic cost to goal
    double f; //total cost
    int speedCount; //number of times remaining to speed

    public Node(Location location, Node parent, double g, double h, int speedCount) {
        this.location = location;
        this.parent = parent;
        this.g = g;
        this.h = h;
        this.f = g + h;
        this.speedCount = speedCount;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Node)) return false;
        Node other = (Node) obj;
        return this.location.equals(other.location) && this.speedCount == other.speedCount;
    }

    @Override
    public int hashCode() {
        return Objects.hash(location, speedCount);
    }
}

class Road {
    private long startId;
    private long endId;
    private int speedLimit;
    private String name;

    //constructor
    public Road(long startId, long endId, int speedLimit, String name) {
        this.startId = startId;
        this.endId = endId;
        this.speedLimit = speedLimit;
        this.name = name;
    }

    public long startId() {
        return startId;
    }

    public long endId() {
        return endId;
    }

    public int speedLimit() {
        return speedLimit;
    }

    public String name() {
        return name;
    }
}

class Location {
    private long id;
    private double latitude;
    private double longitude;

    //constructor
    public Location(long id, double latitude, double longitude) {
        this.id = id;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public long id() {
        return id;
    }

    public double latitude() {
        return latitude;
    }

    public double longitude() {
        return longitude;
    }

    //overrides the "equals()" method to compare two Location objects based on their ID
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Location)) return false;
        Location other = (Location) obj;
        return this.id == other.id;
    }

    @Override
    public int hashCode() {
        return Objects.hash(id);
    }
}

class RoadNetwork {
    //map to store all locations in the network
    private Map<Long, Location> locations = new HashMap<>();
    //adds a new location to the roads network
    private Map<Long, List<Road>> adjacencyList = new HashMap<>();

    public void addLocation(Location loc) {
        locations.put(loc.id(), loc); //adds location to the map
    }

    public Location getLocationForId(long id) {
        return locations.get(id); //gets location based on ID
    }

    //adds road between two locations in the adjacency list
    public void addRoad(Road road) {
        adjacencyList.computeIfAbsent(road.startId(), k -> new ArrayList<>()).add(road);
    }

    //returns a list of roads that are adjacent to a given location
    public List<Road> getAdjacentRoads(long locationId) {
        return adjacencyList.getOrDefault(locationId, Collections.emptyList());
    }
}

class Geometry {
    public static double getDistanceInMiles(Location from, Location to) {
        //haversine formula to calculate distance between two latitude-longitude points
        double R = 3960; //earth radius in miles
        //converts latitude and longitude to radians for both locations
        double lat1 = Math.toRadians(from.latitude());
        double lon1 = Math.toRadians(from.longitude());
        double lat2 = Math.toRadians(to.latitude());
        double lon2 = Math.toRadians(to.longitude());

        //gets difference in lat and long
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;

        //harversine formula
        double a = Math.sin(dlat / 2) * Math.sin(dlat / 2) + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dlon / 2) * Math.sin(dlon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return R * c;
    }

    public static double getDriveTimeInSeconds(Road road, RoadNetwork graph) {
        double distance = getDistanceInMiles(graph.getLocationForId(road.startId()), graph.getLocationForId(road.endId()));
        return (distance / road.speedLimit()) * 3600;
    }
}