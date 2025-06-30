# swa_project_turtlebot
A ROS2 based implementaion of nodes to demonstrate a micro-service based architecture

Presentation: https://docs.google.com/presentation/d/1ya5hnXPa0lHltg59nDbO8mXlAY94eSzh/edit?usp=sharing&ouid=115746902130006592512&rtpof=true&sd=true

## Is ros2 a microservice based architecture?
response from Google's AI:
Yes, ROS 2 can be designed with a microservice architecture. In this approach, individual ROS 2 nodes are treated as independent services, each responsible for a specific function. This allows for improved modularity, maintainability, and scalability, particularly in large-scale robotic systems. 
Here's why ROS 2 is well-suited for microservices:

    Distributed Communication:
    ROS 2's core communication mechanism, based on Data Distribution Service (DDS), is inherently designed for distributed systems, enabling nodes to communicate independently across a network. 

Independent Nodes:
ROS 2 nodes can be developed and deployed independently, making them ideal components for a microservice architecture. 
Service-Oriented Programming:
ROS 2 provides mechanisms for defining and implementing services, allowing nodes to request specific actions from other nodes, similar to how microservices communicate. 
Containerization:
ROS 2 can be effectively deployed using containers like Docker, further simplifying deployment and scaling of microservice-based applications. 

Benefits of using a microservice architecture with ROS 2:

    Improved Modularity:
    Each microservice focuses on a specific task, making the system easier to understand, maintain, and update. 

Increased Scalability:
Microservices can be deployed independently and scaled to meet specific needs, allowing for flexible and responsive systems. 
Fault Isolation:
If one microservice fails, the rest of the system can continue to operate, improving the overall system's reliability. 
Independent Deployment:
Microservices can be updated and deployed independently without affecting the rest of the system, allowing for faster development cycles. 
