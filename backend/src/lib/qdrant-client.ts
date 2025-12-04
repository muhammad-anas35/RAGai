import { QdrantClient } from '@qdrant/js-client-rest';

/**
 * Qdrant vector database client
 */

export interface VectorPoint {
    id: string;
    vector: number[];
    payload: Record<string, any>;
}

export class QdrantClientWrapper {
    private client: QdrantClient;
    private collectionName: string;

    constructor(url: string, apiKey?: string, collectionName: string = 'book_chunks') {
        this.client = new QdrantClient({
            url,
            apiKey
        });
        this.collectionName = collectionName;
    }

    /**
     * Create collection if it doesn't exist
     */
    async ensureCollection(vectorSize: number = 768): Promise<void> {
        try {
            // Check if collection exists
            const collections = await this.client.getCollections();
            const exists = collections.collections.some(c => c.name === this.collectionName);

            if (!exists) {
                // Create collection
                await this.client.createCollection(this.collectionName, {
                    vectors: {
                        size: vectorSize,
                        distance: 'Cosine'
                    },
                    optimizers_config: {
                        default_segment_number: 2
                    },
                    replication_factor: 1
                });
                console.log(`✓ Created collection: ${this.collectionName}`);
            } else {
                console.log(`✓ Collection already exists: ${this.collectionName}`);
            }
        } catch (error) {
            console.error('Error ensuring collection:', error);
            throw error;
        }
    }

    /**
     * Insert vectors in batch
     */
    async upsertVectors(points: VectorPoint[]): Promise<void> {
        try {
            await this.client.upsert(this.collectionName, {
                wait: true,
                points: points.map(p => ({
                    id: p.id,
                    vector: p.vector,
                    payload: p.payload
                }))
            });
        } catch (error) {
            console.error('Error upserting vectors:', error);
            throw error;
        }
    }

    /**
     * Search for similar vectors
     */
    async search(
        queryVector: number[],
        limit: number = 5,
        filter?: Record<string, any>
    ): Promise<Array<{ id: string; score: number; payload: Record<string, any> }>> {
        try {
            const results = await this.client.search(this.collectionName, {
                vector: queryVector,
                limit,
                filter,
                with_payload: true
            });

            return results.map(r => ({
                id: r.id as string,
                score: r.score,
                payload: r.payload as Record<string, any>
            }));
        } catch (error) {
            console.error('Error searching vectors:', error);
            throw error;
        }
    }

    /**
     * Delete collection
     */
    async deleteCollection(): Promise<void> {
        try {
            await this.client.deleteCollection(this.collectionName);
            console.log(`✓ Deleted collection: ${this.collectionName}`);
        } catch (error) {
            console.error('Error deleting collection:', error);
            throw error;
        }
    }

    /**
     * Get collection info
     */
    async getCollectionInfo() {
        try {
            return await this.client.getCollection(this.collectionName);
        } catch (error) {
            console.error('Error getting collection info:', error);
            throw error;
        }
    }

    /**
     * Count vectors in collection
     */
    async countVectors(): Promise<number> {
        try {
            const info = await this.getCollectionInfo();
            return info.points_count || 0;
        } catch (error) {
            console.error('Error counting vectors:', error);
            return 0;
        }
    }
}

// Singleton instance
let qdrantClient: QdrantClientWrapper | null = null;

/**
 * Get or create Qdrant client instance
 */
export function getQdrantClient(): QdrantClientWrapper {
    if (!qdrantClient) {
        const url = process.env.QDRANT_URL || 'http://localhost:6333';
        const apiKey = process.env.QDRANT_API_KEY;
        qdrantClient = new QdrantClientWrapper(url, apiKey);
    }
    return qdrantClient;
}
