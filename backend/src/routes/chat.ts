import { Router, Request, Response } from 'express';
import { executeRAG, executeRAGStream } from '../lib/rag-pipeline';
import { saveMessage, getOrCreateConversation, getUserConversations, getConversationMessages } from '../lib/chat-history';

/**
 * Chat routes for RAG-powered Q&A
 */

const router = Router();

/**
 * POST /api/chat
 * Send a message and get AI response with RAG
 */
router.post('/', async (req: any, res: Response) => {
    try {
        const { message, chapter, conversationId } = req.body;
        const userId = req.userId; // From auth middleware

        // Validation
        if (!message || typeof message !== 'string') {
            return res.status(400).json({
                success: false,
                error: 'Message is required and must be a string'
            });
        }

        if (message.length > 2000) {
            return res.status(400).json({
                success: false,
                error: 'Message must be less than 2000 characters'
            });
        }

        // Get or create conversation
        const convId = conversationId || await getOrCreateConversation(userId);

        // Save user message
        await saveMessage(convId, 'user', message);

        // Execute RAG pipeline
        const result = await executeRAG(message, {
            topK: 5,
            minScore: 0.7,
            chapter: chapter || undefined
        });

        // Save assistant response
        await saveMessage(convId, 'assistant', result.answer, result.sources);

        // Return response
        res.json({
            success: true,
            conversationId: convId,
            message: {
                role: 'assistant',
                content: result.answer,
                sources: result.sources
            },
            metadata: result.metadata
        });

    } catch (error) {
        console.error('Chat error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to generate response. Please try again.'
        });
    }
});

/**
 * POST /api/chat/stream
 * Stream AI response with RAG (for real-time UI)
 */
router.post('/stream', async (req: any, res: Response) => {
    try {
        const { message, chapter } = req.body;

        // Validation
        if (!message || typeof message !== 'string') {
            return res.status(400).json({
                success: false,
                error: 'Message is required'
            });
        }

        // Set headers for SSE (Server-Sent Events)
        res.setHeader('Content-Type', 'text/event-stream');
        res.setHeader('Cache-Control', 'no-cache');
        res.setHeader('Connection', 'keep-alive');

        // Execute RAG pipeline with streaming
        for await (const chunk of executeRAGStream(message, {
            topK: 5,
            minScore: 0.7,
            chapter: chapter || undefined
        })) {
            res.write(`data: ${JSON.stringify(chunk)}\n\n`);
        }

        res.end();

    } catch (error) {
        console.error('Chat stream error:', error);
        res.write(`data: ${JSON.stringify({ type: 'error', data: 'Failed to generate response' })}\n\n`);
        res.end();
    }
});

/**
 * GET /api/chat/history
 * Get user's conversation history
 */
router.get('/history', async (req: any, res: Response) => {
    try {
        const userId = req.userId;
        const limit = parseInt(req.query.limit as string) || 20;
        const offset = parseInt(req.query.offset as string) || 0;

        const conversations = await getUserConversations(userId, limit, offset);

        res.json({
            success: true,
            conversations,
            total: conversations.length,
            limit,
            offset
        });

    } catch (error) {
        console.error('History error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to fetch history'
        });
    }
});

/**
 * GET /api/chat/conversation/:id
 * Get messages for a specific conversation
 */
router.get('/conversation/:id', async (req: any, res: Response) => {
    try {
        const conversationId = req.params.id;
        const messages = await getConversationMessages(conversationId);

        res.json({
            success: true,
            conversationId,
            messages
        });

    } catch (error) {
        console.error('Conversation error:', error);
        res.status(500).json({
            success: false,
            error: 'Failed to fetch conversation'
        });
    }
});

export default router;
