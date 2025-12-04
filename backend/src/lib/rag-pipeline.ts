import { generateQueryEmbedding } from './embeddings';
import { searchSimilarChunks } from './vector-store';
import { getGeminiClient } from './gemini-client';

/**
 * RAG (Retrieval-Augmented Generation) Pipeline
 * Combines vector search with LLM generation
 */

export interface RAGResult {
    answer: string;
    sources: Array<{
        chunkId: string;
        documentPath: string;
        content: string;
        score: number;
        chapter?: string;
        section?: string;
    }>;
    metadata: {
        retrievalTime: number;
        generationTime: number;
        totalTime: number;
        chunksRetrieved: number;
    };
}

export interface RAGOptions {
    topK?: number;           // Number of chunks to retrieve (default: 5)
    minScore?: number;       // Minimum similarity score (default: 0.7)
    chapter?: string;        // Filter by chapter
    maxContextTokens?: number; // Max tokens for context (default: 3000)
}

/**
 * Execute RAG pipeline: retrieve relevant chunks and generate answer
 */
export async function executeRAG(
    query: string,
    options: RAGOptions = {}
): Promise<RAGResult> {
    const startTime = Date.now();

    const {
        topK = 5,
        minScore = 0.7,
        chapter,
        maxContextTokens = 3000
    } = options;

    try {
        // Step 1: Generate query embedding
        console.log('🔍 Generating query embedding...');
        const queryEmbedding = await generateQueryEmbedding(query);

        // Step 2: Search for similar chunks
        console.log('📚 Searching for relevant chunks...');
        const retrievalStart = Date.now();
        const searchResults = await searchSimilarChunks(
            queryEmbedding,
            topK * 2, // Retrieve more, then filter
            chapter
        );
        const retrievalTime = Date.now() - retrievalStart;

        // Step 3: Filter by minimum score
        const filteredResults = searchResults.filter(r => r.score >= minScore);

        if (filteredResults.length === 0) {
            return {
                answer: "I couldn't find relevant information in the book to answer your question. Could you rephrase or ask about a different topic?",
                sources: [],
                metadata: {
                    retrievalTime,
                    generationTime: 0,
                    totalTime: Date.now() - startTime,
                    chunksRetrieved: 0
                }
            };
        }

        // Step 4: Build context from top chunks
        const topChunks = filteredResults.slice(0, topK);
        const context = buildContext(topChunks, maxContextTokens);

        // Step 5: Generate answer with LLM
        console.log('🤖 Generating answer...');
        const generationStart = Date.now();
        const gemini = getGeminiClient();
        const answer = await gemini.generateResponse(query, context);
        const generationTime = Date.now() - generationStart;

        // Step 6: Format sources
        const sources = topChunks.map(chunk => ({
            chunkId: chunk.id,
            documentPath: chunk.documentPath,
            content: chunk.content.substring(0, 200) + '...', // Truncate for response
            score: chunk.score,
            chapter: chunk.chapter,
            section: chunk.section
        }));

        return {
            answer,
            sources,
            metadata: {
                retrievalTime,
                generationTime,
                totalTime: Date.now() - startTime,
                chunksRetrieved: topChunks.length
            }
        };

    } catch (error) {
        console.error('Error in RAG pipeline:', error);
        throw new Error('Failed to generate response. Please try again.');
    }
}

/**
 * Build context string from chunks, respecting token limit
 */
function buildContext(
    chunks: Array<{ content: string; documentPath: string; score: number }>,
    maxTokens: number
): string {
    const contextParts: string[] = [];
    let estimatedTokens = 0;

    for (const chunk of chunks) {
        // Rough token estimation: 1 token ≈ 0.75 words
        const chunkTokens = Math.ceil(chunk.content.split(/\s+/).length / 0.75);

        if (estimatedTokens + chunkTokens > maxTokens) {
            break;
        }

        contextParts.push(
            `[Source: ${chunk.documentPath} (Relevance: ${(chunk.score * 100).toFixed(1)}%)]\n${chunk.content}`
        );
        estimatedTokens += chunkTokens;
    }

    return contextParts.join('\n\n---\n\n');
}

/**
 * Stream RAG response (for real-time UI updates)
 */
export async function* executeRAGStream(
    query: string,
    options: RAGOptions = {}
): AsyncGenerator<{ type: 'sources' | 'chunk' | 'done'; data: any }> {
    const {
        topK = 5,
        minScore = 0.7,
        chapter,
        maxContextTokens = 3000
    } = options;

    try {
        // Step 1-3: Retrieve and filter chunks
        const queryEmbedding = await generateQueryEmbedding(query);
        const searchResults = await searchSimilarChunks(queryEmbedding, topK * 2, chapter);
        const filteredResults = searchResults.filter(r => r.score >= minScore);

        if (filteredResults.length === 0) {
            yield {
                type: 'chunk',
                data: "I couldn't find relevant information to answer your question."
            };
            yield { type: 'done', data: { sources: [] } };
            return;
        }

        const topChunks = filteredResults.slice(0, topK);

        // Yield sources first
        yield {
            type: 'sources',
            data: topChunks.map(chunk => ({
                chunkId: chunk.id,
                documentPath: chunk.documentPath,
                content: chunk.content.substring(0, 200) + '...',
                score: chunk.score,
                chapter: chunk.chapter,
                section: chunk.section
            }))
        };

        // Step 4-5: Generate answer with streaming
        const context = buildContext(topChunks, maxContextTokens);
        const gemini = getGeminiClient();

        for await (const chunk of gemini.generateResponseStream(query, context)) {
            yield { type: 'chunk', data: chunk };
        }

        yield { type: 'done', data: {} };

    } catch (error) {
        console.error('Error in RAG stream:', error);
        throw error;
    }
}
