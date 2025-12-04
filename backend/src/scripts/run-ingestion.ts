#!/usr/bin/env node

import path from 'path';
import { readDocuments, getDocumentStats } from './ingest-content';
import { chunkText, getChunkingStats } from '../lib/chunking';
import { generateEmbeddings } from '../lib/embeddings';
import { storeEmbeddings, getVectorCount } from '../lib/vector-store';

/**
 * Main ingestion orchestrator
 * Reads documents, chunks them, generates embeddings, and stores in Qdrant
 */

async function runIngestion() {
    console.log('🚀 Starting content ingestion pipeline...\n');

    try {
        // 1. Read documents
        console.log('📖 Step 1: Reading documents...');
        const docsDir = path.join(process.cwd(), '..', 'docs', 'physical-ai-book');
        const documents = await readDocuments(docsDir);
        const docStats = getDocumentStats(documents);

        console.log(`✓ Read ${docStats.totalDocuments} documents`);
        console.log(`  Total words: ${docStats.totalWords.toLocaleString()}`);
        console.log(`  Average words per document: ${docStats.averageWordsPerDocument}`);
        console.log(`  Chapters: ${docStats.chapters.join(', ')}\n`);

        // 2. Chunk documents
        console.log('✂️  Step 2: Chunking documents...');
        let allChunks: Array<{
            chunks: any[];
            documentPath: string;
            chapter?: string;
            section?: string;
        }> = [];

        for (const doc of documents) {
            const chunks = chunkText(doc.content, {
                chunkSize: 750,
                chunkOverlap: 100,
                separator: '\n\n',
                keepSeparator: true
            });

            allChunks.push({
                chunks,
                documentPath: doc.path,
                chapter: doc.chapter,
                section: doc.section
            });
        }

        const totalChunks = allChunks.reduce((sum, doc) => sum + doc.chunks.length, 0);
        console.log(`✓ Created ${totalChunks} chunks from ${documents.length} documents\n`);

        // 3. Generate embeddings
        console.log('🧠 Step 3: Generating embeddings...');
        let allEmbeddings: any[] = [];
        let processedChunks = 0;

        for (const doc of allChunks) {
            const embeddings = await generateEmbeddings(
                doc.chunks,
                doc.documentPath,
                doc.chapter,
                doc.section
            );

            allEmbeddings.push(...embeddings);
            processedChunks += doc.chunks.length;

            console.log(`  Progress: ${processedChunks}/${totalChunks} chunks (${Math.round(processedChunks / totalChunks * 100)}%)`);
        }

        console.log(`✓ Generated ${allEmbeddings.length} embeddings\n`);

        // 4. Store in Qdrant
        console.log('💾 Step 4: Storing embeddings in Qdrant...');
        await storeEmbeddings(allEmbeddings);

        const vectorCount = await getVectorCount();
        console.log(`✓ Total vectors in collection: ${vectorCount}\n`);

        // 5. Summary
        console.log('✅ Ingestion complete!');
        console.log('\nSummary:');
        console.log(`  Documents processed: ${documents.length}`);
        console.log(`  Chunks created: ${totalChunks}`);
        console.log(`  Embeddings generated: ${allEmbeddings.length}`);
        console.log(`  Vectors stored: ${vectorCount}`);

    } catch (error) {
        console.error('\n❌ Ingestion failed:', error);
        process.exit(1);
    }
}

// Run if called directly
if (require.main === module) {
    runIngestion()
        .then(() => process.exit(0))
        .catch(error => {
            console.error(error);
            process.exit(1);
        });
}

export { runIngestion };
