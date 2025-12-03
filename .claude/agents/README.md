# Claude Agents for Physical AI RAG System

This directory contains specialized Claude agents for different aspects of the Physical AI textbook and RAG chatbot system.

## Core System Agents

### Authentication Agent
**File**: `authentication_agent.md`
**Purpose**: Implements secure user authentication using better-auth
**Key Responsibilities**:
- Email/password authentication setup
- OAuth social login integration (Google, GitHub)
- Session management with Neon PostgreSQL
- Security best practices (CSRF, rate limiting, email verification)

### Database Agent
**File**: `database_agent.md`
**Purpose**: Manages all database operations with Neon PostgreSQL and Drizzle ORM
**Key Responsibilities**:
- Schema design for users, sessions, chat history
- Type-safe database queries with Drizzle
- Migrations and query optimization
- Connection pooling and performance tuning

### RAG Pipeline Agent
**File**: `rag_pipeline_agent.md`
**Purpose**: Handles retrieval-augmented generation workflow
**Key Responsibilities**:
- Qdrant vector database operations
- Document chunking and embedding generation
- Semantic search and relevance ranking
- Context extraction for LLM prompts

### OpenAI Agent
**File**: `openai_agent.md`
**Purpose**: Integrates OpenAI GPT models for conversational AI
**Key Responsibilities**:
- GPT-4 chat completions
- Streaming responses for better UX
- Function calling and tool integration
- Token management and cost optimization

## Content Creation Agents

### Technical Writer
**File**: `technical_writer.md`
**Purpose**: Writes clear, educational technical content
**Use Cases**: Creating textbook chapters, tutorials, documentation

### Robotics Researcher  
**File**: `robotics_researcher.md`
**Purpose**: Researches latest robotics technologies and best practices
**Use Cases**: Finding academic papers, industry trends, technical specifications

### Robotics Code Generator
**File**: `robotics_code_gen.md`
**Purpose**: Generates ROS 2 Python and C++ code examples
**Use Cases**: Creating working code samples for textbook sections

### Content Manager
**File**: `content_manager.md`
**Purpose**: Organizes and structures textbook content
**Use Cases**: Managing chapters, ensuring consistency, tracking progress

### Domain Expert
**File**: `domain_expert.md`
**Purpose**: Provides deep expertise in Physical AI concepts
**Use Cases**: Reviewing technical accuracy, answering complex questions

## Utility Agents

### Project Navigator
**File**: `project_navigator.md`
**Purpose**: Helps navigate and understand the project structure
**Use Cases**: Finding files, explaining architecture, onboarding

### Code Generator
**File**: `code_generator.md`
**Purpose**: Generates general application code
**Use Cases**: Building React components, API routes, utility functions

### Reviewer
**File**: `reviewer.md`
**Purpose**: Reviews code and content for quality
**Use Cases**: Code reviews, content editing, detecting issues

### Writer
**File**: `writer.md`
**Purpose**: General writing assistance
**Use Cases**: Documentation, README files, user-facing text

## Agent Interaction Flow

```
User Question
    ↓
RAG Pipeline Agent
    ├─→ Generate embedding
    ├─→ Search Qdrant
    └─→ Extract context
        ↓
OpenAI Agent
    ├─→ Build prompt with context
    ├─→ Call GPT-4
    └─→ Stream response
        ↓
Database Agent
    └─→ Save conversation history
```

## Using Agents

### @-mention in Code

Reference specific agents for focused assistance:

```
@authentication_agent How do I implement OAuth with better-auth?
@database_agent Create a schema for storing user preferences
@rag_pipeline_agent What's the best chunking strategy for code documentation?
@openai_agent Implement streaming chat with function calling
```

### Combined Agent Workflows

For complex tasks, agents collaborate:

1. **Building Chat Feature**:
   - `database_agent`: Create chat_history table
   - `authentication_agent`: Verify user session
   - `rag_pipeline_agent`: Search for relevant context
   - `openai_agent`: Generate AI response

2. **Content Ingestion**:
   - `content_manager`: Organize markdown files
   - `rag_pipeline_agent`: Chunk and embed content
   - `database_agent`: Store metadata

3. **New Feature Development**:
   - `project_navigator`: Find relevant files
   - `code_generator`: Build React components
   - `database_agent`: Add required tables
   - `reviewer`: Check code quality

## Configuration

Each agent has:
- **Role definition**: What the agent specializes in
- **Expertise areas**: Specific technologies/skills
- **Task examples**: Common use cases
- **Code patterns**: Reusable snippets
- **Guidelines**: Best practices

## Best Practices

1. **Use specific agents** for their expertise area
2. **Combine agents** for complex, multi-step tasks
3. **Provide context** when asking questions
4. **Reference code patterns** from agent files
5. **Follow guidelines** for consistent quality

## Adding New Agents

To create a new agent:

1. Create `new_agent_name.md` in this directory
2. Follow the template structure:
   - Role
   - Expertise
   - Tasks
   - Code Patterns
   - Guidelines
3. Update this README
4. Use `@new_agent_name` to invoke

## Agent Template

```markdown
# Agent Name

## Role
Clear description of the agent's purpose

## Expertise
- Technology 1
- Technology 2
- Skill area 3

## Tasks
1. **Task Category**
   - Specific task
   - Another task

## Code Patterns
\`\`\`language
// Example code
\`\`\`

## Guidelines
- Best practice 1
- Best practice 2
```

---

**Total Agents**: 16  
**Last Updated**: 2025-12-03
