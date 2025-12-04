import { GoogleGenerativeAI } from '@google/generative-ai';

/**
 * Gemini API client for embeddings and chat
 */

export class GeminiClient {
    private genAI: GoogleGenerativeAI;
    private embeddingModel: any;
    private chatModel: any;

    constructor(apiKey: string) {
        if (!apiKey) {
            throw new Error('GEMINI_API_KEY is required');
        }

        this.genAI = new GoogleGenerativeAI(apiKey);
        this.embeddingModel = this.genAI.getGenerativeModel({ model: 'embedding-001' });
        this.chatModel = this.genAI.getGenerativeModel({
            model: 'gemini-pro',
            generationConfig: {
                temperature: 0.7,
                maxOutputTokens: 1000,
                topP: 0.9,
                topK: 40,
            }
        });
    }

    /**
     * Generate embedding for a single text
     */
    async generateEmbedding(text: string): Promise<number[]> {
        try {
            const result = await this.embeddingModel.embedContent(text);
            return result.embedding.values;
        } catch (error) {
            console.error('Error generating embedding:', error);
            throw error;
        }
    }

    /**
     * Generate embeddings for multiple texts in batch
     */
    async generateEmbeddings(texts: string[]): Promise<number[][]> {
        const embeddings: number[][] = [];

        // Process in batches of 5 to avoid rate limits
        const batchSize = 5;
        for (let i = 0; i < texts.length; i += batchSize) {
            const batch = texts.slice(i, i + batchSize);
            const batchEmbeddings = await Promise.all(
                batch.map(text => this.generateEmbedding(text))
            );
            embeddings.push(...batchEmbeddings);

            // Small delay to avoid rate limiting
            if (i + batchSize < texts.length) {
                await new Promise(resolve => setTimeout(resolve, 100));
            }
        }

        return embeddings;
    }

    /**
     * Generate chat response with context
     */
    async generateResponse(prompt: string, context?: string): Promise<string> {
        try {
            const fullPrompt = context
                ? `Context:\n${context}\n\nQuestion: ${prompt}\n\nAnswer based on the context above:`
                : prompt;

            const result = await this.chatModel.generateContent(fullPrompt);
            const response = await result.response;
            return response.text();
        } catch (error) {
            console.error('Error generating response:', error);
            throw error;
        }
    }

    /**
     * Generate chat response with streaming
     */
    async *generateResponseStream(prompt: string, context?: string): AsyncGenerator<string> {
        try {
            const fullPrompt = context
                ? `Context:\n${context}\n\nQuestion: ${prompt}\n\nAnswer based on the context above:`
                : prompt;

            const result = await this.chatModel.generateContentStream(fullPrompt);

            for await (const chunk of result.stream) {
                const chunkText = chunk.text();
                yield chunkText;
            }
        } catch (error) {
            console.error('Error generating streaming response:', error);
            throw error;
        }
    }
}

// Singleton instance
let geminiClient: GeminiClient | null = null;

/**
 * Get or create Gemini client instance
 */
export function getGeminiClient(): GeminiClient {
    if (!geminiClient) {
        const apiKey = process.env.GEMINI_API_KEY;
        if (!apiKey) {
            throw new Error('GEMINI_API_KEY environment variable is not set');
        }
        geminiClient = new GeminiClient(apiKey);
    }
    return geminiClient;
}
