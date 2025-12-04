import type { ReactNode } from 'react';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';
import styles from './index.module.css';
import HomepageFeatures from '../components/HomepageFeatures';

export default function Home(): ReactNode {
  const history = useHistory();

  const handleGetStarted = () => {
    history.push('/docs/intro');
  };

  const handleLogin = () => {
    history.push('/login');
  };

  const handleSignUp = () => {
    history.push('/signup');
  };

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master the cutting-edge technologies powering the next generation of intelligent robots">
      <div className={styles.heroBanner}>
        <div className={styles.heroContainer}>
          <div className={styles.heroLeft}>
            <p className={styles.heroLabel}>PHYSICAL AI & HUMANOID ROBOTICS</p>
            <h1 className={styles.heroTitle}>Physical AI And Humanoid Robotics</h1>
            <p className={styles.heroAuthor}>By Muhammad Anas Asif</p>
            <p className={styles.heroDescription}>
              Master the cutting-edge technologies powering the next generation of intelligent robots. 
              Learn ROS 2, simulation, NVIDIA Isaac, and conversational AI to build humanoid robots 
              that interact naturally with the physical world.
            </p>
            
            <div className={styles.buttonsSection}>
              <button className={`${styles.button} ${styles.primaryBtn}`} onClick={handleGetStarted}>
                Start Reading
              </button>
              
              <div className={styles.authButtons}>
                <button className={`${styles.button} ${styles.loginBtn}`} onClick={handleLogin}>
                  <span>Login</span>
                </button>
                <button className={`${styles.button} ${styles.signupBtn}`} onClick={handleSignUp}>
                  <span>Sign Up</span>
                </button>
              </div>
            </div>
          </div>
          
          <div className={styles.heroRight}>
            <img 
              src="/img/book-cover.png" 
              alt="Physical AI Book Cover" 
              className={styles.robotImage}
            />
          </div>
        </div>
      </div>
      
      <HomepageFeatures />
    </Layout>
  );
}
