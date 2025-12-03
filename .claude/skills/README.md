# Claude Code Skills for Physical AI RAG Chatbot

This directory contains production-ready Claude Code Skills following the [progressive disclosure pattern](https://www.claude.com/blog/building-skills-for-claude-code).

## 🎯 What are Skills?

Skills are structured knowledge packages that Claude Code can automatically discover and use. Each Skill has:
- **SKILL.md**: Core instructions with YAML frontmatter (name + description)
- **references/**: Detailed documentation loaded only when needed

## 📚 Available Skills

### 1. Neon Database (`neon-database/`)
**Use when**: Working with Neon serverless PostgreSQL database

**Covers**:
- CLI commands (create project, get connection string, branching)
- Drizzle ORM integration and migrations
- Database schemas for users, sessions, chat history
- Query patterns and best practices

**Key Files**:
- `SKILL.md` - Quick start workflow  
- `references/neon-cli.md` - Complete CLI reference
- `references/database-schemas.md` - Schema patterns for RAG chatbot

---

### 2. Qdrant Vector Store (`qdrant-vectordb/`)
**Use when**: Setting up or querying Qdrant vector database

**Covers**:
- Docker and cloud setup
- Collection creation with HNSW indexing
- Embedding generation with OpenAI ada-002
- Semantic search patterns (basic, filtered, hybrid, batch)
- Performance tuning

**Key Files**:
- `SKILL.md` - Quick start workflow
- `references/docker-setup.md` - Local & cloud setup
- `references/search-patterns.md` - 10+ search examples

---

### 3. RAG Pipeline (`rag-pipeline/`)
**Use when**: Building or maintaining the retrieval-augmented generation system

**Covers**:
- Document chunking strategies (800 chars, 200 overlap)
- Content ingestion scripts for Docusaurus markdown
- Embedding generation and vector indexing
- Query flow (question → search → context → GPT-4)
- Context building for LLM prompts

**Key Files**:
- `SKILL.md` - End-to-end RAG workflow
- `references/ingestion-script.md` - Complete ingestion script
- `references/context-building.md` - Building effective context

---

### 4. OpenAI Agents (`openai-agents/`)
**Use when**: Integrating OpenAI GPT models for conversational AI

**Covers**:
- Chat completions with GPT-4 and GPT-3.5-turbo
- Streaming responses for better UX
- Agent personas (Tutor, Code Reviewer, Quick Helper)
- Function calling for tools
- Token management and cost optimization

**Key Files**:
- `SKILL.md` - Agent setup and personas
- `references/streaming.md` - React + SSE streaming examples
- `references/function-calling.md` - Tool integration patterns

---

## 🚀 Using Skills

### In Chat/Code Requests

Claude Code automatically discovers Skills based on context. Just ask naturally:

```
"Set up Neon database for user authentication"
→ Loads neon-database skill

"How do I search Qdrant by chapter?"
→ Loads qdrant-vectordb skill

"Create the RAG ingestion script"
→ Loads rag-pipeline skill

"Stream chat responses with OpenAI"
→ Loads openai-agents skill
```

### Progressive Disclosure

Claude sees lightweight descriptions first (~100 words each). Detailed references load only when needed, keeping context efficient.

**Example flow**:
1. User asks: "Set up Qdrant collection"
2. Claude sees: `qdrant-vectordb` skill description
3. Claude loads: `SKILL.md` (workflow)
4. Claude loads: `references/collections.md` (if needed)

---

## 📂 Skill Structure

```
.claude-skills/
├── neon-database/
│   ├── SKILL.md (workflow + quick ref)
│   └── references/
│       ├── neon-cli.md
│       ├── database-schemas.md
│       ├── drizzle-orm.md
│       ├── query-patterns.md
│       └── migrations.md
│
├── qdrant-vectordb/
│   ├── SKILL.md
│   └── references/
│       ├── docker-setup.md
│       ├── collections.md
│       ├── embeddings.md
│       ├── search-patterns.md
│       └── performance.md
│
├── rag-pipeline/
│   ├── SKILL.md
│   └── references/
│       ├── chunking.md
│       ├── ingestion-script.md
│       ├── query-pipeline.md
│       ├── context-building.md
│       └── error-handling.md
│
└── openai-agents/
    ├── SKILL.md
    └── references/
        ├── streaming.md
        ├── function-calling.md
        ├── tokens.md
        ├── cost-optimization.md
        └── error-handling.md
```

---

## ✅ Best Practices

1. **Update Skills** when adding new patterns or fixing bugs
2. **Keep SKILL.md concise** - detailed info goes in references/
3. **Use metadata** - chapter, section, file path for better search
4. **Test changes** - verify Skills load correctly
5. **Document edge cases** - add to reference files

---

## 🔗 Integration Points

Skills work together:

```
User Question
    ↓
[neon-database] ← Authenticate user
    ↓
[qdrant-vectordb] ← Search textbook
    ↓
[rag-pipeline] ← Build context
    ↓
[openai-agents] ← Generate response
    ↓
[neon-database] ← Save conversation
```

---

## 📖 Learn More

- [Claude Skills Blog Post](https://www.claude.com/blog/building-skills-for-claude-code)
- [Skills Marketplace](https://skillsmp.com/)
- [Implementation Plan](../auth_rag_implementation_plan.md)

---

**Last Updated**: 2025-12-04  
**Total Skills**: 4  
**Total Reference Files**: 20+
