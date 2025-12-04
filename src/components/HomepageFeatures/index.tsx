import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Foundations of Physical AI',
    image: 'https://images.unsplash.com/photo-1677442136019-21780ecad995?w=800&h=600&fit=crop',
    description: (
      <>
        Explore the core principles of embodied intelligence and learn how AI is moving beyond the digital world to interact with physical reality.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics in Depth',
    image: 'https://images.unsplash.com/photo-1485827404703-89b55fcc595e?w=800&h=600&fit=crop',
    description: (
      <>
        Discover the past, present, and future of humanoid robots. Understand their design, mechanics, and the role they will play in our society.
      </>
    ),
  },
  {
    title: 'Hands-On with Simulation',
    image: 'https://images.unsplash.com/photo-1635070041078-e363dbe005cb?w=800&h=600&fit=crop',
    description: (
      <>
        Master the tools of the trade. Learn to design, build, and test robots in realistic simulated environments using Gazebo and NVIDIA Isaac Sim.
      </>
    ),
  },
  {
    title: 'ROS 2 Mastery',
    image: 'https://images.unsplash.com/photo-1518770660439-4636190af475?w=800&h=600&fit=crop',
    description: (
      <>
        Build robust robotic systems with ROS 2. Learn nodes, topics, services, and actions to create modular and scalable robot applications.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac Platform',
    image: 'https://images.unsplash.com/photo-1620712943543-bcc4688e7485?w=800&h=600&fit=crop',
    description: (
      <>
        Leverage NVIDIA's cutting-edge Isaac Sim and Isaac ROS for photorealistic simulation, perception, and AI-powered robot development.
      </>
    ),
  },
  {
    title: 'Conversational AI Integration',
    image: 'https://images.unsplash.com/photo-1531746790731-6c087fecd65a?w=800&h=600&fit=crop',
    description: (
      <>
        Create intelligent robots that understand and respond naturally. Integrate GPT-4, voice recognition, and multimodal AI for human-robot interaction.
      </>
    ),
  },
];

function Feature({ title, image, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={clsx('card card--full-height', styles.featureCard)}>
        <div className={styles.featureImage}>
          <img src={image} alt={title} />
        </div>
        <div className="card__header">
          <Heading as="h3">{title}</Heading>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
