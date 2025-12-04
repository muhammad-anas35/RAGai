import type { VercelRequest, VercelResponse } from '@vercel/node';
import { GoogleGenerativeAI } from '@google/generative-ai';

// Initialize Gemini
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY || '');

export default async function handler(req: VercelRequest, res: VercelResponse) {
    // Handle CORS
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

    if (req.method === 'OPTIONS') {
        return res.status(200).end();
    }

    if (req.method !== 'POST') {
        return res.status(405).json({ error: 'Method not allowed' });
    }

    try {
        const { message } = req.body;

        if (!message) {
            return res.status(400).json({ error: 'Message is required' });
        }

        // Check if Gemini API key is configured
        if (!process.env.GEMINI_API_KEY) {
            return res.status(200).json({
                success: true,
                message: {
                    role: 'assistant',
                    content: `You asked: "${message}". The RAG system is deployed but GEMINI_API_KEY is not configured. Please add it in Vercel Environment Variables.`,
                    sources: []
                }
            });
        }

        // Simple response using Gemini (without RAG for now)
        const model = genAI.getGenerativeModel({ model: 'gemini-pro' });

        const prompt = `You are an AI assistant for a textbook about Physical AI and Humanoid Robotics. 
Answer the following question based on your knowledge about robotics, ROS 2, NVIDIA Isaac, and humanoid robots.
Be helpful, accurate, and concise.

Question: ${message}`;

        const result = await model.generateContent(prompt);
        const response = result.response.text();

        return res.status(200).json({
            success: true,
            message: {
                role: 'assistant',
                content: response,
                sources: []
            },
            metadata: {
                model: 'gemini-pro',
                timestamp: new Date().toISOString()
            }
        });
    } catch (error: any) {
        console.error('Chat error:', error);
        return res.status(500).json({
            success: false,
            error: error.message || 'Failed to process message'
        });
    }
}
