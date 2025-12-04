# Writing Style Guide for Technical Writer

## Tone and Voice

### Characteristics
- **Clear**: Use simple, direct language
- **Educational**: Explain concepts step-by-step
- **Encouraging**: Foster confidence in readers
- **Professional**: Maintain technical accuracy
- **Accessible**: Avoid unnecessary jargon

### Example
❌ **Bad**: "ROS 2 leverages DDS middleware to facilitate inter-process communication paradigms."

✅ **Good**: "ROS 2 uses DDS middleware to help different programs communicate with each other."

## Structure

### Chapter Layout
```markdown
# Chapter Title

## Introduction
Brief overview (2-3 paragraphs)

## Section 1: Core Concept
Detailed explanation with examples

### Subsection: Practical Application
Hands-on implementation

## Section 2: Advanced Topics
Build on previous knowledge

## Summary
Key takeaways (bullet points)

## Exercises
Hands-on practice problems
```

### Code Examples
Always include:
1. **Comments** explaining each section
2. **Complete** runnable code
3. **Expected output** in comments
4. **Common errors** and solutions

Example:
```python
# Create a ROS 2 publisher node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('hello_publisher')
        
        # Create publisher that sends String messages
        # 'hello_topic' is the topic name
        # 10 is the queue size
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
    def publish_message(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

# Expected output:
# [INFO] [hello_publisher]: Published: Hello, ROS 2!
```

## Formatting Guidelines

### Headings
- **H1 (#)**: Chapter titles only
- **H2 (##)**: Major sections
- **H3 (###)**: Subsections
- **H4 (####)**: Rarely; for very specific details

### Lists
**Use bulleted lists for**:
- Unordered items
- Features
- Benefits

**Use numbered lists for**:
1. Sequential steps
2. Procedures
3. Ranked items

### Emphasis
- **Bold**: Key terms on first introduction
- *Italic*: Emphasis or technical terms
- `Code`: Function names, variables, commands

### Callouts
Use admonitions for important information:

```markdown
:::tip
Use ROS 2 Humble for long-term support!
:::

:::caution
Never run `sudo` commands without understanding them.
:::

:::info
Additional resources available at docs.ros.org
:::
```

## Technical Accuracy Checklist

- [ ] All code examples tested and run successfully
- [ ] Commands include correct syntax
- [ ] File paths use proper conventions
- [ ] Version numbers specified where relevant
- [ ] Screenshots/diagrams are up-to-date
- [ ] Links verified and working

## Common Patterns

### Introducing New Concepts
1. **What**: Define the concept
2. **Why**: Explain its importance
3. **How**: Show practical implementation
4. **When**: Describe use cases

### Example:
> **What is a ROS 2 Node?**
> A node is a single program that performs a specific task in a robot system.
>
> **Why Use Nodes?**
> Nodes allow you to break complex robotics software into smaller, manageable pieces.
>
> **How to Create a Node:**
> ```python
> from rclpy.node import Node
> class MyNode(Node):
>     def __init__(self):
>         super().__init__('my_node')
> ```
>
> **When to Use Nodes:**
> Use separate nodes for distinct tasks like sensor reading, motor control, and planning.

## Accessibility

### Reading Level
- Target: Undergraduate computer science students
- Use **active voice** (90%+ of sentences)
- Keep sentences under 25 words
- Break complex ideas into multiple paragraphs

### Code Accessibility
- Include **beginner**, **intermediate**, and **advanced** examples
- Provide **commented** and **uncommented** versions
- Link to **video tutorials** when available

## Quality Standards

### Before Publishing
- [ ] Spell check completed
- [ ] Grammar check passed
- [ ] Technical review by domain expert
- [ ] Code examples tested on target ROS 2 version
- [ ] Links verified
- [ ] Images optimized (<500KB)
- [ ] Consistent terminology throughout

### Metrics
- **Clarity**: Can a beginner understand?
- **Completeness**: Are all steps included?
- **Correctness**: Is everything technically accurate?
- **Conciseness**: Is it as short as possible without losing clarity?

## Resources

- **Style Guide**: Google Developer Documentation Style Guide
- **Grammar**: Grammarly, Hemingway Editor
- **Code Formatting**: Black (Python), clang-format (C++)
- **Markdown Linting**: markdownlint
