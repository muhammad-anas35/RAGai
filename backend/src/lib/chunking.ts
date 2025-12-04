/**
 * Text chunking utilities for RAG pipeline
 * Splits documents into optimal-sized chunks for embedding
 */

export interface ChunkOptions {
    chunkSize: number;      // Target chunk size in tokens (approx)
    chunkOverlap: number;   // Overlap between chunks in tokens
    separator: string;      // Text separator (e.g., "\n\n" for paragraphs)
    keepSeparator: boolean; // Whether to keep separator in chunks
}

export interface TextChunk {
    content: string;
    index: number;
    wordCount: number;
}

const DEFAULT_OPTIONS: ChunkOptions = {
    chunkSize: 750,        // ~750 tokens per chunk
    chunkOverlap: 100,     // ~100 token overlap
    separator: '\n\n',     // Split on paragraph breaks
    keepSeparator: true
};

/**
 * Estimate token count from word count (rough approximation)
 * Average: 1 token ≈ 0.75 words
 */
function estimateTokens(text: string): number {
    const words = text.split(/\s+/).filter(w => w.length > 0).length;
    return Math.ceil(words / 0.75);
}

/**
 * Split text into chunks based on separator
 */
function splitBySeparator(text: string, separator: string, keepSeparator: boolean): string[] {
    if (!separator) {
        return [text];
    }

    const parts = text.split(separator);

    if (keepSeparator && parts.length > 1) {
        // Re-add separator to each part (except last)
        return parts.map((part, i) =>
            i < parts.length - 1 ? part + separator : part
        ).filter(part => part.trim().length > 0);
    }

    return parts.filter(part => part.trim().length > 0);
}

/**
 * Chunk text into optimal-sized pieces
 */
export function chunkText(text: string, options: Partial<ChunkOptions> = {}): TextChunk[] {
    const opts = { ...DEFAULT_OPTIONS, ...options };
    const chunks: TextChunk[] = [];

    // Split text by separator (e.g., paragraphs)
    const parts = splitBySeparator(text, opts.separator, opts.keepSeparator);

    let currentChunk = '';
    let currentTokens = 0;
    let chunkIndex = 0;

    for (let i = 0; i < parts.length; i++) {
        const part = parts[i];
        const partTokens = estimateTokens(part);

        // If single part exceeds chunk size, split it further
        if (partTokens > opts.chunkSize) {
            // Save current chunk if not empty
            if (currentChunk.trim().length > 0) {
                chunks.push({
                    content: currentChunk.trim(),
                    index: chunkIndex++,
                    wordCount: currentChunk.split(/\s+/).filter(w => w.length > 0).length
                });
                currentChunk = '';
                currentTokens = 0;
            }

            // Split large part by sentences
            const sentences = part.split(/(?<=[.!?])\s+/);
            let sentenceChunk = '';
            let sentenceTokens = 0;

            for (const sentence of sentences) {
                const sentenceTokenCount = estimateTokens(sentence);

                if (sentenceTokens + sentenceTokenCount > opts.chunkSize && sentenceChunk.length > 0) {
                    chunks.push({
                        content: sentenceChunk.trim(),
                        index: chunkIndex++,
                        wordCount: sentenceChunk.split(/\s+/).filter(w => w.length > 0).length
                    });

                    // Add overlap from previous chunk
                    const overlapText = sentenceChunk.split(/\s+/).slice(-opts.chunkOverlap).join(' ');
                    sentenceChunk = overlapText + ' ' + sentence;
                    sentenceTokens = estimateTokens(sentenceChunk);
                } else {
                    sentenceChunk += (sentenceChunk ? ' ' : '') + sentence;
                    sentenceTokens += sentenceTokenCount;
                }
            }

            if (sentenceChunk.trim().length > 0) {
                currentChunk = sentenceChunk;
                currentTokens = sentenceTokens;
            }
        } else {
            // Check if adding this part exceeds chunk size
            if (currentTokens + partTokens > opts.chunkSize && currentChunk.length > 0) {
                // Save current chunk
                chunks.push({
                    content: currentChunk.trim(),
                    index: chunkIndex++,
                    wordCount: currentChunk.split(/\s+/).filter(w => w.length > 0).length
                });

                // Start new chunk with overlap
                const overlapText = currentChunk.split(/\s+/).slice(-opts.chunkOverlap).join(' ');
                currentChunk = overlapText + (overlapText ? ' ' : '') + part;
                currentTokens = estimateTokens(currentChunk);
            } else {
                // Add to current chunk
                currentChunk += (currentChunk ? (opts.keepSeparator ? '' : ' ') : '') + part;
                currentTokens += partTokens;
            }
        }
    }

    // Add final chunk
    if (currentChunk.trim().length > 0) {
        chunks.push({
            content: currentChunk.trim(),
            index: chunkIndex,
            wordCount: currentChunk.split(/\s+/).filter(w => w.length > 0).length
        });
    }

    return chunks;
}

/**
 * Get chunking statistics
 */
export function getChunkingStats(chunks: TextChunk[]) {
    const totalChunks = chunks.length;
    const totalWords = chunks.reduce((sum, chunk) => sum + chunk.wordCount, 0);
    const avgWords = totalChunks > 0 ? Math.round(totalWords / totalChunks) : 0;
    const minWords = totalChunks > 0 ? Math.min(...chunks.map(c => c.wordCount)) : 0;
    const maxWords = totalChunks > 0 ? Math.max(...chunks.map(c => c.wordCount)) : 0;

    return {
        totalChunks,
        totalWords,
        averageWordsPerChunk: avgWords,
        minWordsPerChunk: minWords,
        maxWordsPerChunk: maxWords
    };
}
