import numpy as np
from .policy import softmax
from .NN import ActionValueNetwork
class Adam():
    def __init__(self, layer_sizes, 
                 optimizer_info):
        self.layer_sizes = layer_sizes

        # Specify Adam algorithm's hyper parameters
        self.step_size = optimizer_info.get("step_size")
        self.beta_m = optimizer_info.get("beta_m")
        self.beta_v = optimizer_info.get("beta_v")
        self.epsilon = optimizer_info.get("epsilon")
        
        # Initialize Adam algorithm's m and v
        self.m = [dict() for i in range(1, len(self.layer_sizes))]
        self.v = [dict() for i in range(1, len(self.layer_sizes))]
        
        for i in range(0, len(self.layer_sizes) - 1):
            
            self.m[i]["W"] = np.zeros((self.layer_sizes[i], self.layer_sizes[i + 1]))
            self.m[i]["b"] = np.zeros((1, self.layer_sizes[i + 1]))
            self.v[i]["W"] = np.zeros((self.layer_sizes[i], self.layer_sizes[i + 1]))
            self.v[i]["b"] = np.zeros((1, self.layer_sizes[i + 1]))            
    
        self.beta_m_product = self.beta_m
        self.beta_v_product = self.beta_v
    
    def update_weights(self, weights, td_errors_times_gradients):
        """
        Args:
            weights (Array of dictionaries): The weights of the neural network.
            td_errors_times_gradients (Array of dictionaries): The gradient of the 
            action-values with respect to the network's weights times the TD-error
        Returns:
            The updated weights (Array of dictionaries).
        """
        for i in range(len(weights)):
            for param in weights[i].keys():
                weight_update = None
                
                g = td_errors_times_gradients
                self.m[i][param] = self.beta_m * self.m[i][param] + (1 - self.beta_m) * g[i][param]
                self.v[i][param] = self.beta_v * self.v[i][param] + (1 - self.beta_v) * (g[i][param] * g[i][param])
                
                m_hat = self.m[i][param] / (1 - self.beta_m_product)
                v_hat = self.v[i][param] / (1 - self.beta_v_product)
                
                weight_update = self.step_size * m_hat / (np.sqrt(v_hat) + self.epsilon)
                
                weights[i][param] = weights[i][param] + weight_update

        self.beta_m_product *= self.beta_m
        self.beta_v_product *= self.beta_v
        
        return weights
    
def get_td_error(states, next_states, actions, rewards, discount, terminals,
                network : ActionValueNetwork,
                current_q : ActionValueNetwork,
                tau):
    """
    Args:
        states (Numpy array): The batch of states with the shape (batch_size, state_dim).
        next_states (Numpy array): The batch of next states with the shape (batch_size, state_dim).
        actions (Numpy array): The batch of actions with the shape (batch_size,).
        rewards (Numpy array): The batch of rewards with the shape (batch_size,).
        discount (float): The discount factor.
        terminals (Numpy array): The batch of terminals with the shape (batch_size,).
        network (ActionValueNetwork): The latest state of the network that is getting replay updates.
        current_q (ActionValueNetwork): The fixed network used for computing the targets, 
                                        and particularly, the action-values at the next-states.
    Returns:
        The TD errors (Numpy array) for actions taken, of shape (batch_size,)
    """    
    # Compute action values at next states using current_q network
    # Note that q_next_mat is a 2D array of shape (batch_size, num_actions)
    q_next_mat = current_q.get_action_values(next_states)
    
    # Compute policy at next state by passing the action-values in q_next_mat to softmax()
    # Note that probs_mat is a 2D array of shape (batch_size, num_actions)
    probs_mat = softmax(q_next_mat, tau)
    
    # Compute the estimate of the next state value, v_next_vec.
    v_next_vec = np.sum(probs_mat*q_next_mat, axis=1)*(1-terminals)
    
    # Compute Expected Sarsa target
    # Note that target_vec is a 1D array of shape (batch_size,)
    target_vec = rewards + discount * v_next_vec
    
    # Compute action values at the current states for all actions using network
    # Note that q_mat is a 2D array of shape (batch_size, num_actions)
    q_mat = network.get_action_values(states)
    
    # Batch Indices is an array from 0 to the batch size - 1. 
    batch_indices = np.arange(q_mat.shape[0])

    # Compute q_vec by selecting q(s, a) from q_mat for taken actions
    q_vec = q_mat[batch_indices, actions]
    
    # Compute TD errors for actions taken
    delta_vec = target_vec - q_vec
    
    return delta_vec